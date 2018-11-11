/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * MarlinSerial.cpp - Hardware serial library for Wiring
 * Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
 *
 * Modified 23 November 2006 by David A. Mellis
 * Modified 28 September 2010 by Mark Sproul
 * Modified 14 February 2016 by Andreas Hardtung (added tx buffer)
 */

#include "MarlinSerial.h"
#include "Marlin.h"

// Disable HardwareSerial.cpp to support chips without a UART (Attiny, etc.)

#if !defined(USBCON) && (defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H))

#ifdef USED_SERIAL_MAX
uint8_t MarlinSerial::portIndex = 0;
#endif

#ifdef USED_SERIAL_MAX
	#if UART_PRESENT(SERIAL_PORT)
	ring_buffer_r rx_buffer[USED_SERIAL_MAX + 1] = { 0 };
	#if TX_BUFFER_SIZE > 0
	ring_buffer_t tx_buffer[USED_SERIAL_MAX + 1] = { 0 };
	static bool _written;
	#endif
	#endif
#else
  #if UART_PRESENT(SERIAL_PORT)
    ring_buffer_r rx_buffer = { { 0 }, 0, 0 };
    #if TX_BUFFER_SIZE > 0
      ring_buffer_t tx_buffer = { { 0 }, 0, 0 };
      static bool _written;
    #endif
  #endif
#endif

  #if ENABLED(EMERGENCY_PARSER)

    #include "stepper.h"
    #include "language.h"

    // Currently looking for: M108, M112, M410
    // If you alter the parser please don't forget to update the capabilities in Conditionals_post.h

    FORCE_INLINE void emergency_parser(const unsigned char c) {

      static e_parser_state state = state_RESET;

      switch (state) {
        case state_RESET:
          switch (c) {
            case ' ': break;
            case 'N': state = state_N;      break;
            case 'M': state = state_M;      break;
            default: state = state_IGNORE;
          }
          break;

        case state_N:
          switch (c) {
            case '0': case '1': case '2':
            case '3': case '4': case '5':
            case '6': case '7': case '8':
            case '9': case '-': case ' ':   break;
            case 'M': state = state_M;      break;
            default:  state = state_IGNORE;
          }
          break;

        case state_M:
          switch (c) {
            case ' ': break;
            case '1': state = state_M1;     break;
            case '4': state = state_M4;     break;
            default: state = state_IGNORE;
          }
          break;

        case state_M1:
          switch (c) {
            case '0': state = state_M10;    break;
            case '1': state = state_M11;    break;
            default: state = state_IGNORE;
          }
          break;

        case state_M10:
          state = (c == '8') ? state_M108 : state_IGNORE;
          break;

        case state_M11:
          state = (c == '2') ? state_M112 : state_IGNORE;
          break;

        case state_M4:
          state = (c == '1') ? state_M41 : state_IGNORE;
          break;

        case state_M41:
          state = (c == '0') ? state_M410 : state_IGNORE;
          break;

        case state_IGNORE:
          if (c == '\n') state = state_RESET;
          break;

        default:
          if (c == '\n') {
            switch (state) {
              case state_M108:
                wait_for_user = wait_for_heatup = false;
                break;
              case state_M112:
                kill(PSTR(MSG_KILLED));
                break;
              case state_M410:
                quickstop_stepper();
                break;
              default:
                break;
            }
            state = state_RESET;
          }
      }
    }

  #endif // EMERGENCY_PARSER
#ifdef USED_SERIAL_MAX
	FORCE_INLINE void store_char(unsigned char c, int index) {
		CRITICAL_SECTION_START;
		uint8_t h = rx_buffer[index].head,
			i = (uint8_t)(h + 1)  & (RX_BUFFER_SIZE - 1);
		// if we should be storing the received character into the location
		// just before the tail (meaning that the head would advance to the
		// current location of the tail), we're about to overflow the buffer
		// and so we don't write the character or advance the head.
		if (i != rx_buffer[index].tail) {
			rx_buffer[index].buffer[h] = c;
			rx_buffer[index].head = i;
		}
		CRITICAL_SECTION_END;
	#if ENABLED(EMERGENCY_PARSER)
			emergency_parser(c);
	#endif
	}
#else
  FORCE_INLINE void store_char(unsigned char c) {
    CRITICAL_SECTION_START;
      const uint8_t h = rx_buffer.head,
                    i = (uint8_t)(h + 1) & (RX_BUFFER_SIZE - 1);

      // if we should be storing the received character into the location
      // just before the tail (meaning that the head would advance to the
      // current location of the tail), we're about to overflow the buffer
      // and so we don't write the character or advance the head.
      if (i != rx_buffer.tail) {
        rx_buffer.buffer[h] = c;
        rx_buffer.head = i;
      }
    CRITICAL_SECTION_END;

    #if ENABLED(EMERGENCY_PARSER)
      emergency_parser(c);
    #endif
  }
#endif

  #if TX_BUFFER_SIZE > 0
#ifdef USED_SERIAL_MAX
	#ifdef SERIAL_PORT
	FORCE_INLINE void _tx_udr_empty_irq0() {
		// If interrupts are enabled, there must be more data in the output
		// buffer. Send the next byte
		uint8_t t = tx_buffer[SERIAL_PORT].tail;
		uint8_t c = tx_buffer[SERIAL_PORT].buffer[t];
		tx_buffer[SERIAL_PORT].tail = (t + 1) & (TX_BUFFER_SIZE - 1);
		M_UDRx(SERIAL_PORT) = c;
		// clear the TXC bit -- "can be cleared by writing a one to its bit
		// location". This makes sure flush() won't return until the bytes
		// actually got written
		SBI(M_UCSRxA(SERIAL_PORT), M_TXCx(SERIAL_PORT));
		if (tx_buffer[SERIAL_PORT].head == tx_buffer[SERIAL_PORT].tail) {
			CBI(M_UCSRxB(SERIAL_PORT), M_UDRIEx(SERIAL_PORT));// Buffer empty, so disable interrupts
		}
	}
	#endif

	#ifdef SERIAL1_PORT
	FORCE_INLINE void _tx_udr_empty_irq1() {
		// If interrupts are enabled, there must be more data in the output
		// buffer. Send the next byte
		uint8_t t = tx_buffer[SERIAL1_PORT].tail;
		uint8_t c = tx_buffer[SERIAL1_PORT].buffer[t];
		tx_buffer[SERIAL1_PORT].tail = (t + 1) & (TX_BUFFER_SIZE - 1);
		M_UDRx(SERIAL1_PORT) = c;
		// clear the TXC bit -- "can be cleared by writing a one to its bit
		// location". This makes sure flush() won't return until the bytes
		// actually got written
		SBI(M_UCSRxA(SERIAL1_PORT), M_TXCx(SERIAL1_PORT));
		if (tx_buffer[SERIAL1_PORT].head == tx_buffer[SERIAL1_PORT].tail) {
			CBI(M_UCSRxB(SERIAL1_PORT), M_UDRIEx(SERIAL1_PORT));// Buffer empty, so disable interrupts
		}
	}
	#endif

	#ifdef SERIAL2_PORT
	FORCE_INLINE void _tx_udr_empty_irq2() {
		// If interrupts are enabled, there must be more data in the output
		// buffer. Send the next byte
		uint8_t t = tx_buffer[SERIAL2_PORT].tail;
		uint8_t c = tx_buffer[SERIAL2_PORT].buffer[t];
		tx_buffer[SERIAL2_PORT].tail = (t + 1) & (TX_BUFFER_SIZE - 1);
		M_UDRx(SERIAL2_PORT)=c;
		// clear the TXC bit -- "can be cleared by writing a one to its bit
		// location". This makes sure flush() won't return until the bytes
		// actually got written
		SBI(M_UCSRxA(SERIAL2_PORT), M_TXCx(SERIAL2_PORT));
		if (tx_buffer[SERIAL2_PORT].head == tx_buffer[SERIAL2_PORT].tail) {
			CBI(M_UCSRxB(SERIAL2_PORT), M_UDRIEx(SERIAL2_PORT));// Buffer empty, so disable interrupts
		}
	}
	#endif

	#ifdef SERIAL3_PORT
	FORCE_INLINE void _tx_udr_empty_irq3() {
		// If interrupts are enabled, there must be more data in the output
		// buffer. Send the next byte
		uint8_t t = tx_buffer[SERIAL3_PORT].tail;
		uint8_t c = tx_buffer[SERIAL3_PORT].buffer[t];
		tx_buffer[SERIAL3_PORT].tail = (t + 1) & (TX_BUFFER_SIZE - 1);
		M_UDRx(SERIAL3_PORT)=c;
		// clear the TXC bit -- "can be cleared by writing a one to its bit
		// location". This makes sure flush() won't return until the bytes
		// actually got written
		SBI(M_UCSRxA(SERIAL3_PORT), M_TXCx(SERIAL3_PORT));
		if (tx_buffer[SERIAL3_PORT].head == tx_buffer[SERIAL3_PORT].tail) {
			CBI(M_UCSRxB(SERIAL3_PORT), M_UDRIEx(SERIAL3_PORT));// Buffer empty, so disable interrupts
		}
	}
	#endif
	#ifdef SERIAL_PORT
	#ifdef M_USARTx_UDRE_vect(SERIAL_PORT)
		ISR(M_USARTx_UDRE_vect(SERIAL_PORT)) {
			_tx_udr_empty_irq0();
		}
	#endif
	#endif

	#ifdef SERIAL1_PORT
	#ifdef M_USARTx_UDRE_vect(SERIAL1_PORT)
		ISR(M_USARTx_UDRE_vect(SERIAL1_PORT)) {
			_tx_udr_empty_irq1();
		}
	#endif
	#endif

	#ifdef SERIAL2_PORT
	#ifdef M_USARTx_UDRE_vect(SERIAL2_PORT)
		ISR(M_USARTx_UDRE_vect(SERIAL2_PORT)) {
			_tx_udr_empty_irq2();
		}
	#endif
	#endif

	#ifdef SERIAL3_PORT
	#ifdef M_USARTx_UDRE_vect(SERIAL3_PORT)
		ISR(M_USARTx_UDRE_vect(SERIAL3_PORT)) {
			_tx_udr_empty_irq3();
		}
	#endif
	#endif
#else //marlin default
    FORCE_INLINE void _tx_udr_empty_irq(void) {
      // If interrupts are enabled, there must be more data in the output
      // buffer. Send the next byte
      const uint8_t t = tx_buffer.tail,
                    c = tx_buffer.buffer[t];
      tx_buffer.tail = (t + 1) & (TX_BUFFER_SIZE - 1);

      M_UDRx = c;

      // clear the TXC bit -- "can be cleared by writing a one to its bit
      // location". This makes sure flush() won't return until the bytes
      // actually got written
      SBI(M_UCSRxA, M_TXCx);

      if (tx_buffer.head == tx_buffer.tail) {
        // Buffer empty, so disable interrupts
        CBI(M_UCSRxB, M_UDRIEx);
      }
    }

    #ifdef M_USARTx_UDRE_vect
      ISR(M_USARTx_UDRE_vect) {
        _tx_udr_empty_irq();
      }
    #endif
#endif //USED_SERIAL_MAX
  #endif // TX_BUFFER_SIZE

#ifdef USED_SERIAL_MAX
	#ifdef SERIAL_PORT
	#ifdef M_USARTx_RX_vect(SERIAL_PORT)
		ISR(M_USARTx_RX_vect(SERIAL_PORT)) {
			unsigned char c = M_UDRx(SERIAL_PORT);
			store_char(c,SERIAL_PORT);
		}
	#endif
	#endif
	#ifdef SERIAL1_PORT
	#ifdef M_USARTx_RX_vect(SERIAL1_PORT)
		ISR(M_USARTx_RX_vect(SERIAL1_PORT)) {
			unsigned char c = M_UDRx(SERIAL1_PORT);
			store_char(c, SERIAL1_PORT);
		}
	#endif
	#endif
	#ifdef SERIAL2_PORT
	#ifdef M_USARTx_RX_vect(SERIAL2_PORT)
		ISR(M_USARTx_RX_vect(SERIAL2_PORT)) {
			unsigned char c = M_UDRx(SERIAL2_PORT);
			store_char(c, SERIAL2_PORT);
		}
	#endif
	#endif
	#ifdef SERIAL3_PORT
	#ifdef M_USARTx_RX_vect(SERIAL3_PORT)
		ISR(M_USARTx_RX_vect(SERIAL3_PORT)) {
			unsigned char c = M_UDRx(SERIAL3_PORT);
			store_char(c, SERIAL3_PORT);
		}
	#endif
	#endif
#else //marlin default
  #ifdef M_USARTx_RX_vect
    ISR(M_USARTx_RX_vect) {
      const unsigned char c = M_UDRx;
      store_char(c);
    }
  #endif
#endif

  // Public Methods
#ifdef USED_SERIAL_MAX
void MarlinSerial::begin(long baud) {

	#define	baud_setting(x) baud_setting##x
	#define	_baud_setting(x) baud_setting(x)
	#define useU2X(x) useU2X##x
	#define _useU2X(x) useU2X(x)

	#ifdef SERIAL_PORT
		uint16_t _baud_setting(SERIAL_PORT);
		bool _useU2X(SERIAL_PORT) = true;
	#endif
	#ifdef SERIAL1_PORT
		uint16_t _baud_setting(SERIAL1_PORT);
		bool _useU2X(SERIAL1_PORT) = true;
	#endif
	#ifdef SERIAL2_PORT
		uint16_t _baud_setting(SERIAL2_PORT);
		bool _useU2X(SERIAL2_PORT) = true;
	#endif
	#ifdef SERIAL3_PORT
		uint16_t _baud_setting(SERIAL3_PORT);
		bool _useU2X(SERIAL3_PORT) = true;
	#endif

	#if F_CPU == 16000000UL&&((defined(SERIAL_PORT)&& SERIAL_PORT== 0)||\
		(defined(SERIAL1_PORT) && SERIAL1_PORT == 0) || \
		(defined(SERIAL2_PORT) && SERIAL2_PORT == 0) || \
		(defined(SERIAL3_PORT) && SERIAL3_PORT == 0))
		// hard-coded exception for compatibility with the bootloader shipped
		// with the Duemilanove and previous boards and the firmware on the 8U2
		// on the Uno and Mega 2560.
		if (baud == 57600) {
	#if defined(SERIAL_PORT)&& SERIAL_PORT== 0
			_useU2X(SERIAL_PORT) = false;
	#endif
	#if defined(SERIAL1_PORT)&& SERIAL1_PORT== 0
			_useU2X(SERIAL1_PORT) = false;
	#endif
	#if defined(SERIAL2_PORT)&& SERIAL2_PORT== 0
			_useU2X(SERIAL2_PORT) = false;
	#endif
	#if defined(SERIAL3_PORT)&& SERIAL3_PORT== 0
			_useU2X(SERIAL3_PORT) = false;
	#endif
		}
	#endif

	#define	_SERIAL_INIT(portx) \
		if (useU2X(portx))\
		{\
		M_UCSRxA(portx) = _BV(M_U2Xx(portx)); \
		baud_setting(portx) = (F_CPU / 4 / baud - 1) / 2; \
		}\
		else \
		{\
		M_UCSRxA(portx) = 0; \
		baud_setting(portx) = (F_CPU / 8 / baud - 1) / 2; \
		}\
		M_UBRRxH(portx) = baud_setting(portx) >> 8; \
		M_UBRRxL(portx) = baud_setting(portx); \
		SBI(M_UCSRxB(portx), M_RXENx(portx)); \
		SBI(M_UCSRxB(portx), M_TXENx(portx)); \
		SBI(M_UCSRxB(portx), M_RXCIEx(portx))

	#ifdef SERIAL_PORT
		_SERIAL_INIT(SERIAL_PORT);
	#endif

	#ifdef SERIAL1_PORT
		_SERIAL_INIT(SERIAL1_PORT);
	#endif

	#ifdef SERIAL2_PORT
		_SERIAL_INIT(SERIAL2_PORT);
	#endif

	#ifdef SERIAL3_PORT
		_SERIAL_INIT(SERIAL3_PORT);
	#endif

	#if TX_BUFFER_SIZE > 0
	#ifdef SERIAL_PORT
		CBI(M_UCSRxB(SERIAL_PORT), M_UDRIEx(SERIAL_PORT));
	#endif

	#ifdef SERIAL1_PORT
		CBI(M_UCSRxB(SERIAL1_PORT), M_UDRIEx(SERIAL1_PORT));
	#endif

	#ifdef SERIAL2_PORT
		CBI(M_UCSRxB(SERIAL2_PORT), M_UDRIEx(SERIAL2_PORT));
	#endif

	#ifdef SERIAL3_PORT
		CBI(M_UCSRxB(SERIAL3_PORT), M_UDRIEx(SERIAL3_PORT));
	#endif
		_written = false;
	#endif
	}
#else //marlin default
  void MarlinSerial::begin(const long baud) {
    uint16_t baud_setting;
    bool useU2X = true;

    #if F_CPU == 16000000UL && SERIAL_PORT == 0
      // hard-coded exception for compatibility with the bootloader shipped
      // with the Duemilanove and previous boards and the firmware on the 8U2
      // on the Uno and Mega 2560.
      if (baud == 57600) useU2X = false;
    #endif

    if (useU2X) {
      M_UCSRxA = _BV(M_U2Xx);
      baud_setting = (F_CPU / 4 / baud - 1) / 2;
    }
    else {
      M_UCSRxA = 0;
      baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }

    // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
    M_UBRRxH = baud_setting >> 8;
    M_UBRRxL = baud_setting;

    SBI(M_UCSRxB, M_RXENx);
    SBI(M_UCSRxB, M_TXENx);
    SBI(M_UCSRxB, M_RXCIEx);
    #if TX_BUFFER_SIZE > 0
      CBI(M_UCSRxB, M_UDRIEx);
      _written = false;
    #endif
  }
#endif

#ifdef USED_SERIAL_MAX
	void MarlinSerial::end() {
	#define _SHUT_PORT(portx) \
		CBI(M_UCSRxB(portx), M_RXENx(portx)); \
		CBI(M_UCSRxB(portx), M_TXENx(portx)); \
		CBI(M_UCSRxB(portx), M_RXCIEx(portx)); \
		CBI(M_UCSRxB(portx), M_UDRIEx(portx))
	#ifdef SERIAL_PORT
		_SHUT_PORT(SERIAL_PORT);
	#endif
	#ifdef SERIAL1_PORT
		_SHUT_PORT(SERIAL1_PORT);
	#endif
	#ifdef SERIAL2_PORT
		_SHUT_PORT(SERIAL2_PORT);
	#endif
	#ifdef SERIAL3_PORT
		_SHUT_PORT(SERIAL3_PORT);
	#endif
	}
#else
  void MarlinSerial::end() {
    CBI(M_UCSRxB, M_RXENx);
    CBI(M_UCSRxB, M_TXENx);
    CBI(M_UCSRxB, M_RXCIEx);
    CBI(M_UCSRxB, M_UDRIEx);
  }
#endif

#ifdef USED_SERIAL_MAX
void MarlinSerial::checkRx(void) {
#define _CHECKRX(portx) \
	if (TEST(M_UCSRxA(portx), M_RXCx(portx))) {\
	uint8_t c = M_UDRx(portx); \
	store_char(c, portx); \
	}
	#ifdef SERIAL_PORT
	_CHECKRX(SERIAL_PORT)
	#endif
	#ifdef SERIAL1_PORT
		_CHECKRX(SERIAL1_PORT)
	#endif
	#ifdef SERIAL2_PORT
		_CHECKRX(SERIAL2_PORT)
	#endif
	#ifdef SERIAL3_PORT
		_CHECKRX(SERIAL3_PORT)
	#endif
	}
#else
  void MarlinSerial::checkRx(void) {
    if (TEST(M_UCSRxA, M_RXCx)) {
      const uint8_t c = M_UDRx;
      store_char(c);
    }
  }
#endif

#ifdef USED_SERIAL_MAX
  inline char checkPort(uint8_t pindex)
  {
	  char port;
	  for (port = 0; port < 8; port++)if (pindex&_BV(port))break;
	  return port;
  }
  int MarlinSerial::peek() {
	  char port = checkPort(portIndex);
	  if (port == 8)return -1;
	  CRITICAL_SECTION_START;
	  int v = rx_buffer[port].head == rx_buffer[port].tail ? -1 : rx_buffer[port].buffer[rx_buffer[port].tail];
	  CRITICAL_SECTION_END;
	  return v;
  }
#else
  int MarlinSerial::peek(void) {
    CRITICAL_SECTION_START;
      const int v = rx_buffer.head == rx_buffer.tail ? -1 : rx_buffer.buffer[rx_buffer.tail];
    CRITICAL_SECTION_END;
    return v;
  }
#endif

#ifdef USED_SERIAL_MAX
  int MarlinSerial::read() {
	  char portx = checkPort(portIndex);
	  if (portx == 8)return 0;
	  uint8_t t;
	  int v = -1;
	  CRITICAL_SECTION_START;
	  t = rx_buffer[portx].tail;
	  if (rx_buffer[portx].head != t)
	  {
		  v = rx_buffer[portx].buffer[t];
		  rx_buffer[portx].tail = (uint8_t)(t + 1) & (RX_BUFFER_SIZE - 1);
	  }
	  CRITICAL_SECTION_END;
	  return v;
  }
#else
  int MarlinSerial::read(void) {
    int v;
    CRITICAL_SECTION_START;
      const uint8_t t = rx_buffer.tail;
      if (rx_buffer.head == t)
        v = -1;
      else {
        v = rx_buffer.buffer[t];
        rx_buffer.tail = (uint8_t)(t + 1) & (RX_BUFFER_SIZE - 1);
      }
    CRITICAL_SECTION_END;
    return v;
  }
#endif

#ifdef USED_SERIAL_MAX
  uint8_t MarlinSerial::available() {
	  char portx = checkPort(portIndex);
	  if (portx == 8)return 0;
	  uint8_t h, t = 0;
	  CRITICAL_SECTION_START;
	  h = rx_buffer[portx].head;
	  t = rx_buffer[portx].tail;
	  t = (uint8_t)(RX_BUFFER_SIZE + h - t) & (RX_BUFFER_SIZE - 1);
	  CRITICAL_SECTION_END;
	  return t;
  }
#else
  uint8_t MarlinSerial::available(void) {
    CRITICAL_SECTION_START;
      const uint8_t h = rx_buffer.head,
                    t = rx_buffer.tail;
    CRITICAL_SECTION_END;
    return (uint8_t)(RX_BUFFER_SIZE + h - t) & (RX_BUFFER_SIZE - 1);
  }
#endif

#ifdef USED_SERIAL_MAX
  void MarlinSerial::flush() {
	  // RX
	  // don't reverse this or there may be problems if the RX interrupt
	  // occurs after reading the value of rx_buffer_head but before writing
	  // the value to rx_buffer_tail; the previous value of rx_buffer_head
	  // may be written to rx_buffer_tail, making it appear as if the buffer
	  // were full, not empty.
	  char port = checkPort(portIndex);
	  if (port == 8)return;
	  CRITICAL_SECTION_START;
	  rx_buffer[port].head = rx_buffer[port].tail;
	  CRITICAL_SECTION_END;
  }
#else
  void MarlinSerial::flush(void) {
    // RX
    // don't reverse this or there may be problems if the RX interrupt
    // occurs after reading the value of rx_buffer_head but before writing
    // the value to rx_buffer_tail; the previous value of rx_buffer_head
    // may be written to rx_buffer_tail, making it appear as if the buffer
    // were full, not empty.
    CRITICAL_SECTION_START;
      rx_buffer.head = rx_buffer.tail;
    CRITICAL_SECTION_END;
  }
#endif

#if TX_BUFFER_SIZE > 0
	#ifdef USED_SERIAL_MAX
	  uint8_t MarlinSerial::availableForWrite() {
		  char port = checkPort(portIndex);
		  if (port == 8)return 0xFF;
		  CRITICAL_SECTION_START;
		  uint8_t h = tx_buffer[port].head;
		  uint8_t t = tx_buffer[port].tail;
		  CRITICAL_SECTION_END;
		  return (uint8_t)(TX_BUFFER_SIZE + h - t) & (TX_BUFFER_SIZE - 1);
	  }
	#else
		uint8_t MarlinSerial::availableForWrite(void) {
		  CRITICAL_SECTION_START;
			const uint8_t h = tx_buffer.head,
						  t = tx_buffer.tail;
		  CRITICAL_SECTION_END;
		  return (uint8_t)(TX_BUFFER_SIZE + h - t) & (TX_BUFFER_SIZE - 1);
		}
	#endif

	#ifdef USED_SERIAL_MAX
	  void MarlinSerial::write(uint8_t c) {
		  _written = true;
		  uint8_t index = portIndex; 
		  unsigned char _sreg;
	#define CRITICAL_SECTION_BEGIN _sreg = SREG; cli()
	#define CRITICAL_SECTION_STOP	SREG=_sreg
		  // If the buffer and the data register is empty, just write the byte
		  // to the data register and be done. This shortcut helps
		  // significantly improve the effective datarate at high (>
		  // 500kbit/s) bitrates, where interrupt overhead becomes a slowdown.
	#define BUFF_TEST(portx) (tx_buffer[portx].head == tx_buffer[portx].tail)
	#define _testAndSendUDR(portx) \
		CRITICAL_SECTION_BEGIN; \
		bool tf = BUFF_TEST(portx);\
		CRITICAL_SECTION_STOP; \
		  if (tf) {\
		  if(TEST(M_UCSRxA(portx), M_UDREx(portx))) {\
		  CRITICAL_SECTION_BEGIN; \
		  M_UDRx(portx) = c; \
		  SBI(M_UCSRxA(portx), M_TXCx(portx)); \
		  CRITICAL_SECTION_STOP; \
		  index&=~_BV(portx); }\
		  }
	#ifdef SERIAL_PORT
		  if (index&_BV(SERIAL_PORT))
		  {
			  _testAndSendUDR(SERIAL_PORT);
		  }
	#endif
	#ifdef SERIAL1_PORT
		  if (index&_BV(SERIAL1_PORT))
		  {
			  _testAndSendUDR(SERIAL1_PORT);
		  }
	#endif
	#ifdef SERIAL2_PORT
		  if (index&_BV(SERIAL2_PORT))
		  {
			  _testAndSendUDR(SERIAL2_PORT);
		  }
	#endif
	#ifdef SERIAL3_PORT
		  if (index&_BV(SERIAL3_PORT))
		  {
			  _testAndSendUDR(SERIAL3_PORT);
		  }
	#endif

		  if (index == 0)return;

	#define _fun(x)	_tx_udr_empty_irq##x
	#define fun(x) _fun(x)
		  /*If the output buffer is full, there's nothing for it other than to wait for the interrupt handler to empty it a bit*/
		  /*Interrupts are disabled, so we'll have to poll the data register empty flag ourselves. If it is set, pretend an interrupt has happened and call the handler to free up space for us.*/
		  /*nop, the interrupt handler will free up space for us*/
		  uint8_t i;
	#define BUFFER_SEND(portx) \
		  if (index&_BV(portx))\
		  {\
		  i= (tx_buffer[portx].head + 1) & (TX_BUFFER_SIZE - 1); \
		  while (i == tx_buffer[portx].tail) {\
		  if (!TEST(SREG, SREG_I)) {\
		  if (TEST(M_UCSRxA(SERIAL_PORT), M_UDREx(SERIAL_PORT))) \
		  fun(portx)();} \
		  else {}}\
		  }tx_buffer[portx].buffer[tx_buffer[portx].head] = c;\
		  CRITICAL_SECTION_BEGIN; \
		  tx_buffer[portx].head = i;\
		  SBI(M_UCSRxB(portx), M_UDRIEx(portx));\
		  CRITICAL_SECTION_STOP

	#ifdef SERIAL_PORT
		  BUFFER_SEND(SERIAL_PORT);
	#endif
	#ifdef SERIAL1_PORT
		  BUFFER_SEND(SERIAL1_PORT);
	#endif
	#ifdef SERIAL2_PORT
		  BUFFER_SEND(SERIAL2_PORT);
	#endif
	#ifdef SERIAL3_PORT
		  BUFFER_SEND(SERIAL3_PORT);
	#endif
		  return;
	  }
	#else
		void MarlinSerial::write(const uint8_t c) {
		  _written = true;
		  CRITICAL_SECTION_START;
			bool emty = (tx_buffer.head == tx_buffer.tail);
		  CRITICAL_SECTION_END;
		  // If the buffer and the data register is empty, just write the byte
		  // to the data register and be done. This shortcut helps
		  // significantly improve the effective datarate at high (>
		  // 500kbit/s) bitrates, where interrupt overhead becomes a slowdown.
		  if (emty && TEST(M_UCSRxA, M_UDREx)) {
			CRITICAL_SECTION_START;
			  M_UDRx = c;
			  SBI(M_UCSRxA, M_TXCx);
			CRITICAL_SECTION_END;
			return;
		  }
		  const uint8_t i = (tx_buffer.head + 1) & (TX_BUFFER_SIZE - 1);

		  // If the output buffer is full, there's nothing for it other than to
		  // wait for the interrupt handler to empty it a bit
		  while (i == tx_buffer.tail) {
			if (!TEST(SREG, SREG_I)) {
			  // Interrupts are disabled, so we'll have to poll the data
			  // register empty flag ourselves. If it is set, pretend an
			  // interrupt has happened and call the handler to free up
			  // space for us.
			  if (TEST(M_UCSRxA, M_UDREx))
				_tx_udr_empty_irq();
			}
			else {
			  // nop, the interrupt handler will free up space for us
			}
		  }

		  tx_buffer.buffer[tx_buffer.head] = c;
		  { CRITICAL_SECTION_START;
			  tx_buffer.head = i;
			  SBI(M_UCSRxB, M_UDRIEx);
			CRITICAL_SECTION_END;
		  }
		  return;
		}
	#endif

	#ifdef USED_SERIAL_MAX
	  void MarlinSerial::flushTX(void) {
		  // TX
		  // If we have never written a byte, no need to flush. This special
		  // case is needed since there is no way to force the TXC (transmit
		  // complete) bit to 1 during initialization
		  if (!_written)
			  return;

	#ifdef SERIAL_PORT
		  while (TEST(M_UCSRxB(SERIAL_PORT), M_UDRIEx(SERIAL_PORT)) || !TEST(M_UCSRxA(SERIAL_PORT), M_TXCx(SERIAL_PORT))) {
			  if (!TEST(SREG, SREG_I) && TEST(M_UCSRxB(SERIAL_PORT), M_UDRIEx(SERIAL_PORT)))
				  // Interrupts are globally disabled, but the DR empty
				  // interrupt should be enabled, so poll the DR empty flag to
				  // prevent deadlock
			  if (TEST(M_UCSRxA(SERIAL_PORT), M_UDREx(SERIAL_PORT)))
				  _tx_udr_empty_irq0();
		  }
	#endif
		  // If we get here, nothing is queued anymore (DRIE is disabled) and
		  // the hardware finished tranmission (TXC is set).
	#ifdef SERIAL1_PORT
		  while (TEST(M_UCSRxB(SERIAL1_PORT), M_UDRIEx(SERIAL1_PORT)) || !TEST(M_UCSRxA(SERIAL1_PORT), M_TXCx(SERIAL1_PORT))) {
			  if (!TEST(SREG, SREG_I) && TEST(M_UCSRxB(SERIAL1_PORT), M_UDRIEx(SERIAL1_PORT)))
			  if (TEST(M_UCSRxA(SERIAL1_PORT), M_UDREx(SERIAL1_PORT)))
				  _tx_udr_empty_irq1();
		  }
	#endif
	#ifdef SERIAL2_PORT
		  while (TEST(M_UCSRxB(SERIAL2_PORT), M_UDRIEx(SERIAL2_PORT)) || !TEST(M_UCSRxA(SERIAL2_PORT), M_TXCx(SERIAL2_PORT))) {
			  if (!TEST(SREG, SREG_I) && TEST(M_UCSRxB(SERIAL2_PORT), M_UDRIEx(SERIAL2_PORT)))
			  if (TEST(M_UCSRxA(SERIAL2_PORT), M_UDREx(SERIAL2_PORT)))
				  _tx_udr_empty_irq2();
		  }
	#endif
	#ifdef SERIAL3_PORT
		  while (TEST(M_UCSRxB(SERIAL3_PORT), M_UDRIEx(SERIAL3_PORT)) || !TEST(M_UCSRxA(SERIAL3_PORT), M_TXCx(SERIAL3_PORT))) {
			  if (!TEST(SREG, SREG_I) && TEST(M_UCSRxB(SERIAL3_PORT), M_UDRIEx(SERIAL3_PORT)))
			  if (TEST(M_UCSRxA(SERIAL3_PORT), M_UDREx(SERIAL3_PORT)))
				  _tx_udr_empty_irq1();
		  }
	#endif
	  }
	#else
		void MarlinSerial::flushTX(void) {
		  // TX
		  // If we have never written a byte, no need to flush. This special
		  // case is needed since there is no way to force the TXC (transmit
		  // complete) bit to 1 during initialization
		  if (!_written)
			return;

		  while (TEST(M_UCSRxB, M_UDRIEx) || !TEST(M_UCSRxA, M_TXCx)) {
			if (!TEST(SREG, SREG_I) && TEST(M_UCSRxB, M_UDRIEx))
			  // Interrupts are globally disabled, but the DR empty
			  // interrupt should be enabled, so poll the DR empty flag to
			  // prevent deadlock
			  if (TEST(M_UCSRxA, M_UDREx))
				_tx_udr_empty_irq();
		  }
		  // If we get here, nothing is queued anymore (DRIE is disabled) and
		  // the hardware finished tranmission (TXC is set).
	  }
	#endif
#else
	#ifdef USED_SERIAL_MAX
	void MarlinSerial::write(uint8_t c) {
	#define _writer(portx) \
		while (!TEST(M_UCSRxA(portx), M_UDREx(portx)));\
		M_UDRx(portx) = c

	#ifdef SERIAL_PORT
		if(portIndex&_BV(SERIAL_PORT))
		{
			_writer(SERIAL_PORT);
		}
	#endif
	#ifdef SERIAL1_PORT
		if(portIndex&_BV(SERIAL1_PORT))
		{
			_writer(SERIAL1_PORT);
		}
	#endif
	#ifdef SERIAL2_PORT
		if (portIndex&_BV(SERIAL2_PORT))
		{
			_writer(SERIAL2_PORT);
		}
	#endif
	#ifdef SERIAL3_PORT
		if (portIndex&_BV(SERIAL3_PORT))
		{
			_writer(SERIAL3_PORT);
		}
	#endif
	}
	#else
		void MarlinSerial::write(uint8_t c) {
		  while (!TEST(M_UCSRxA, M_UDREx))
			;
		  M_UDRx = c;
		}
	#endif
#endif

  // end NEW

  /// imports from print.h


  void MarlinSerial::print(char c, int base) {
    print((long)c, base);
  }

  void MarlinSerial::print(unsigned char b, int base) {
    print((unsigned long)b, base);
  }

  void MarlinSerial::print(int n, int base) {
    print((long)n, base);
  }

  void MarlinSerial::print(unsigned int n, int base) {
    print((unsigned long)n, base);
  }

  void MarlinSerial::print(long n, int base) {
    if (base == 0)
      write(n);
    else if (base == 10) {
      if (n < 0) {
        print('-');
        n = -n;
      }
      printNumber(n, 10);
    }
    else
      printNumber(n, base);
  }

  void MarlinSerial::print(unsigned long n, int base) {
    if (base == 0) write(n);
    else printNumber(n, base);
  }

  void MarlinSerial::print(double n, int digits) {
    printFloat(n, digits);
  }

  void MarlinSerial::println(void) {
    print('\r');
    print('\n');
  }

  void MarlinSerial::println(const String& s) {
    print(s);
    println();
  }

  void MarlinSerial::println(const char c[]) {
    print(c);
    println();
  }

  void MarlinSerial::println(char c, int base) {
    print(c, base);
    println();
  }

  void MarlinSerial::println(unsigned char b, int base) {
    print(b, base);
    println();
  }

  void MarlinSerial::println(int n, int base) {
    print(n, base);
    println();
  }

  void MarlinSerial::println(unsigned int n, int base) {
    print(n, base);
    println();
  }

  void MarlinSerial::println(long n, int base) {
    print(n, base);
    println();
  }

  void MarlinSerial::println(unsigned long n, int base) {
    print(n, base);
    println();
  }

  void MarlinSerial::println(double n, int digits) {
    print(n, digits);
    println();
  }

  // Private Methods

  void MarlinSerial::printNumber(unsigned long n, uint8_t base) {
    if (n) {
      unsigned char buf[8 * sizeof(long)]; // Enough space for base 2
      int8_t i = 0;
      while (n) {
        buf[i++] = n % base;
        n /= base;
      }
      while (i--)
        print((char)(buf[i] + (buf[i] < 10 ? '0' : 'A' - 10)));
    }
    else
      print('0');
  }

  void MarlinSerial::printFloat(double number, uint8_t digits) {
    // Handle negative numbers
    if (number < 0.0) {
      print('-');
      number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i)
      rounding *= 0.1;

    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    print(int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits) {
      print('.');
      // Extract digits from the remainder one at a time
      while (digits--) {
        remainder *= 10.0;
        int toPrint = int(remainder);
        print(toPrint);
        remainder -= toPrint;
      }
    }
  }

  // Preinstantiate
  MarlinSerial customizedSerial;

#endif // !USBCON && (UBRRH || UBRR0H || UBRR1H || UBRR2H || UBRR3H)

// For AT90USB targets use the UART for BT interfacing
#if defined(USBCON) && ENABLED(BLUETOOTH)
  HardwareSerial bluetoothSerial;
#endif
