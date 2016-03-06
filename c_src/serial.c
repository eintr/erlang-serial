/*
Copyright (c) 1996, 1999 Johan Bevemyr
Copyright (c) 2007, 2009 Tony Garnock-Jones

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
/*    -*- C -*- 
 *    File:	 serial.c  (~jb/serialport/serial.c)
 *    Author:	 Johan Bevemyr
 *    Created:	 Fri Oct 18 09:59:34 1996
 *    Purpose:   Provide Erlang with access to the serial port.
 */ 

/* This program communicates through the following protocoll when
 * run with -erlang:
 *
 *   (all messages starts with a two byte header containing the
 *   size of the message)
 *
 *   SEND DATA
 *        Transmits DATA on the currently open tty
 * 
 *   CONNECT     
 *        Reopens a disconnected connection.
 *
 *   DISCONNECT
 *        Hangs up an open serial line
 * 
 *   OPEN path 
 *        Opens a new tty, closing the old (but not disconnecting). It
 *        is possible to alternate between two connections. 
 *
 *   SPEED inspeed outspeed
 *        Sets the input and output speeds. New connections will
 *        have these speeds. It is also possible to change speed
 *        on the fly.
 *
 *   PARITY ODD/EVEN
 *        Sets the parity of the current connection.
 *
 *   BREAK
 *        Sends break.
 *   
 *   usage: serial [-debug] [-cbreak] [-erlang] [-speed <bit rate>] [-tty <dev>]
 *          bit rate is one of 
 *                  50      75      110
 *                  134     150     200
 *                  300     600     1200
 *                  1800    2400    4800
 *                  9600    19200   38400
 *                  57600   115200  230400
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <ctype.h>
#include <unistd.h>
#include <errno.h>

#include "serial.h"

bit_rate bitrate_table[] = {
  {0      , B0	   },
  {50     , B50	   },
  {75     , B75	   },
  {110    , B110   },
  {134    , B134   },
  {150    , B150   },
  {200    , B200   },
  {300    , B300   },
  {600    , B600   },
  {1200   , B1200  },
  {1800   , B1800  },
  {2400   , B2400  },
  {4800   , B4800  },
  {9600   , B9600  },
  {19200  , B19200 },	
  {38400  , B38400 },	
  {57600  , B57600 },	
  {115200 , B115200 }, 	
  {230400 , B230400 },
  {-1, B0}
};

#define	DATASIZE	65536

struct erl_msg_st {
	uint8_t function_id;
	uint16_t datalen;
	uint8_t data[DATASIZE];
};

/**********************************************************************
 * Name: get_speed
 *
 * Desc: Returns the speed_t value associated with a given bit_rate
 *       according to the bitrate_table. B0 is returned if no matching entry 
 *       is found.
 */
speed_t get_speed(int speed)
{
	int i;

	for(i=0; bitrate_table[i].rate>0; i++) {
		if (speed == bitrate_table[i].rate)	break;
	}
	return bitrate_table[i].speed;
}
  
/**********************************************************************
 * Name: set_raw_tty_mode
 *
 * Desc: Configures the given tty for raw-mode.
 */
void set_raw_tty_mode(int fd)
{
	struct termios ttymodes;

	/* Get ttymodes */

	if (tcgetattr(fd,&ttymodes)<0) {
		perror("tcgetattr");
		exit(1);
	}

	/* Configure for raw mode (see man termios) */
	ttymodes.c_cc[VMIN] = 1;         /* at least one character */
	ttymodes.c_cc[VTIME] = 0;        /* do not wait to fill buffer */

	ttymodes.c_iflag &= ~(ICRNL |    /* disable CR-to-NL mapping */
			INLCR |    /* disable NL-to-CR mapping */
			IGNCR |    /* disable ignore CR */
			ISTRIP |   /* disable stripping of eighth bit */
			IXON |     /* disable output flow control */
			BRKINT |   /* disable generate SIGINT on brk */
			IGNPAR |
			PARMRK |
			IGNBRK |
			INPCK);    /* disable input parity detection */

	ttymodes.c_lflag &= ~(ICANON |   /* enable non-canonical mode */
			ECHO |     /* disable character echo */
			ECHOE |    /* disable visual erase */
			ECHOK |    /* disable echo newline after kill */
			ECHOKE |   /* disable visual kill with bs-sp-bs */
			ECHONL |   /* disable echo nl when echo off */
			ISIG | 	   /* disable tty-generated signals */
			IEXTEN);   /* disable extended input processing */

	ttymodes.c_cflag |= CS8;         /* enable eight bit chars */
	ttymodes.c_cflag &= ~PARENB;     /* disable input parity check */

	ttymodes.c_oflag &= ~OPOST;      /* disable output processing */

	/* roland */
	ttymodes.c_cflag |= CLOCAL;


	/* Apply changes */

	if (tcsetattr(fd, TCSAFLUSH, &ttymodes) < 0) {
		perror("tcsetattr");
		exit(1);
	}
}

/**********************************************************************
 * Name: set_tty_speed
 *
 * Desc: set input and output speeds of a given connection.
 */
void set_tty_speed(int fd, speed_t new_ispeed, speed_t new_ospeed)
{
	struct termios ttymodes;

	/* Get ttymodes */

	if (tcgetattr(fd,&ttymodes) < 0) {
		perror("tcgetattr");
		exit(1);
	}

	if (cfsetispeed(&ttymodes,new_ispeed) < 0) {
		perror("cfsetispeed");
		exit(1);
	}

	if (cfsetospeed(&ttymodes,new_ospeed) < 0) {
		perror("cfsetospeed");
		exit(1);
	}

	ttymodes.c_cflag |= CRTSCTS;     /* enable RTS/CTS flow control */

	/* Apply hanges */

	if (tcsetattr(fd, TCSAFLUSH, &ttymodes) < 0) {
		perror("tcsetattr");
		exit(1);
	}
}

/**********************************************************************
 * Name: get_tbh_size
 * Desc: returns the size of a two_byte_header message (from Erlang).
 */
int get_tbh_size(unsigned char buf[2]) {
  return (buf[0] << 8) | buf[1];
}

/**********************************************************************
 * Name: set_tbh_size
 * Desc: sets the first two bytes of the buffer to its size
 */
void set_tbh_size(unsigned char buf[2], int size) {
  buf[1] = (unsigned char) (size & 0xff);
  buf[0] = (unsigned char) ((size >> 8) & 0xff);
  return;
}

/**********************************************************************
 * Name: tbh_write
 * Desc: writes the buffer to a file descriptor, adding size info
 *       at the beginning.
 */
void tbh_write(unsigned char buf[], int buffsize) {
	struct iovec iov[2];
	unsigned char header_buf[TBHSIZE];

	Debug1("tbh_write: send message of size %d\r\n", buffsize);

	/* First, write two byte header */
	iov[0].iov_base = header_buf;
	iov[0].iov_len = TBHSIZE;
	/* Second, write original buffer */
	iov[1].iov_base = buf;
	iov[1].iov_len = buffsize;

	set_tbh_size(header_buf, buffsize);

	writev(STDOUT_FILENO, iov, 2);

	return;
}

/**********************************************************************
 * Name: readn(fd,buf,nr)
 * Desc: Try to read nr bytes and put them into buf. Return the number
 *       of bytes read, i.e. nr.
 * Returns: The number of bytes read, or 0 if stream closed.
 */
int readn(int fd, unsigned char *buf, int nr)
{
	int pos = 0;
	ssize_t ret;

	while(nr > 0) {
		ret = read(fd, buf+pos, nr);
		if (ret<0) {
			perror("read()");
			break;
		} else if (ret == 0) {
			break;
		} else {
			pos += ret;
			nr -= ret;
		}
	}
	return pos;
}

/**********************************************************************
 * Name: tbh_read
 * Desc: Reads one message with two-byte-header, filling buffer.
 *       Returns the number of elements used in the buffer, or 0
 *       if the input file has been closed. 
 *
 */
int tbh_read(int fd, struct erl_msg_st *msg)
{
	union {
		uint16_t datasize;
		uint8_t datasize_byte[2];
	} c;
	ssize_t ret;

	if (read(fd, c.datasize_byte+1, 1) < 1) return 0;
	if (read(fd, c.datasize_byte+0, 1) < 1) return 0;
	if (read(fd, &msg->function_id, 1)!=1) return 0;

	c.datasize--;
	msg->datalen=0;
	while (c.datasize>0) {
		ret = readn(fd, msg->data + msg->datalen, c.datasize);
		if (ret==0) {
			break;
		} else if (ret<0) {
			perror("read()");
			break;
		} else {
			msg->datalen += ret;
			c.datasize -= ret;
		}
	}
	msg->data[msg->datalen] = 0;

	Debug1("tbh_read: got message of size %d\r\n", msg->datalen);

	return msg->datalen;
}

/**********************************************************************
 * Name: write_to_tty
 * Desc: write a number of bytes found in the buffer to the tty, 
 *       filling the buffer from the given fillfd if neccessary.
 *
 */
void write_to_tty(int fd, uint8_t *buf, size_t len) {
	union {
		uint16_t datasize;
		uint8_t datasize_byte[2];
	} c;
	size_t pos;
	ssize_t ret;

	c.datasize = len;
	if (write(fd, c.datasize_byte+1, 1) < 1) return;
	if (write(fd, c.datasize_byte+0, 1) < 1) return;

	pos = 0;
	while (len>0) {
		ret = write(fd, buf+pos, len);
		if (ret<0) {
			perror("write()");
			break;
		} else {
			pos += ret;
			len -= ret;
		}
	}
	return;
}

/**********************************************************************/

int Debug_Enabled = FALSE;

int main(int argc, char *argv[])
{
	int            ttyfd = -1;           /* terminal file descriptor */
	speed_t        in_speed=B9600;       /* default in speed         */
	speed_t        out_speed=B9600;      /* default out speed        */
	char           ttyname[MAXPATHLEN];  /* terminal name            */

	Debug_Enabled = TRUE;

	/****************************************
	 * Start processing loop
	 */

	{
		fd_set readfds;           /* file descriptor bit field for select */
		int    maxfd;             /* max file descriptor for select */
		unsigned char buf[MAXLENGTH];    /* buffer for transfer between serial-user */

		/* Set up initial bit field for select */
		maxfd = Max(STDIN_FILENO, ttyfd);

		while (TRUE)
		{
			int i;

			FD_ZERO(&readfds);

			FD_SET(STDIN_FILENO, &readfds);
			if(ttyfd != -1) FD_SET(ttyfd, &readfds);

			i = select(maxfd+1, &readfds, NULL, NULL, NULL);

			if (i <= 0)
			{
				perror("select");
				exit(1);
			}

			/******************************
			 * Data from TTY
			 */
			if (ttyfd != -1 && FD_ISSET(ttyfd, &readfds)) {
				int nr_read;

				Debug("receiving from TTY\r\n");

				FD_CLR(ttyfd,&readfds);

				nr_read = read(ttyfd,buf,MAXLENGTH);

				if (nr_read <= 0)
				{
					fprintf(stderr,"problem reading from tty\n");
					exit(1);
				}

				tbh_write(buf, nr_read);
			}

			/******************************
			 * Data from controlling process
			 */
			if (FD_ISSET(STDIN_FILENO,&readfds)) {
				int nr_read;
				struct erl_msg_st msg;

				/* Messages from Erlang are structured as:
				 *   Length:16
				 *   PacketType:8
				 *   DATA
				 */

				nr_read = tbh_read(STDIN_FILENO, &msg);

				/* Check if stdin closed, i.e. controlling
				 * process terminated.
				 */
				if (nr_read == 0)
					exit(1);

				/* Interpret packets from Erlang
				 */
				switch(msg.function_id) {
					case SEND:	   /******************************/
						Debug("received SEND\r\n");
						if (ttyfd!=-1) {
							write_to_tty(ttyfd, msg.data, msg.datalen);
						}
						break;

					case OPEN:	   /******************************/
						Debug("received OPEN ");
						/* Terminate string */
						buf[nr_read] = '\0';
						strcpy(ttyname, (void*)(msg.data));
					case CONNECT:    /******************************/
						Debug1("(Re)connecting %s \r\n",ttyname);

						if (ttyfd!=-1) close(ttyfd);

						ttyfd = open(ttyname, O_RDWR | O_NOCTTY);
						if (ttyfd==-1) {
							fprintf(stderr,"Cannot open terminal %s.\n",
									&buf[HEADERSIZE]);
							exit(1);
						}

						set_raw_tty_mode(ttyfd);
						set_tty_speed(ttyfd, in_speed, out_speed);
						break;

					case DISCONNECT: /******************************/
						Debug("received DISCONNECT\r\n");
						if (ttyfd!=-1) set_tty_speed(ttyfd,B0,B0);
					case CLOSE:	   /******************************/
						Debug("CLOSE\r\n");
						if (ttyfd!=-1) close(ttyfd);
						ttyfd = -1;
						break;

					case SPEED:	   /******************************/
						{
							int speed;
							speed = get_speed(strtol((void*)(msg.data), NULL, 10));

							Debug1("     raw SPEED %s\r\n",&buf[HEADERSIZE]);
							Debug2("received SPEED %ud %ud\r\n",
									(unsigned int) in_speed,
									(unsigned int) out_speed);

							if(ttyfd!=-1)
								set_tty_speed(ttyfd, speed, speed);
							break;
						}

					case PARITY_ODD: /******************************/
						break;

					case PARITY_EVEN:/******************************/
						break;

					case BREAK:      /******************************/
						if (ttyfd!=-1)
							(void) tcsendbreak(ttyfd,BREAKPERIOD);
						break;

					default:
						fprintf(stderr,"%s: unknown command from Erlang\n",
								argv[0]);
						break;
				}
			}
		}
	}
	return 0;
}
