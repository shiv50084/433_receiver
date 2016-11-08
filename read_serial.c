/***************************************************************************
 *   Copyright (C) 2009 by Valentin Manea                                  *
 *   valentin.manea@gmail.com                                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include	<string.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<signal.h>
#include	<limits.h>
#include        <sys/select.h>
#include	<pthread.h>
#include	<errno.h>
#include	<fcntl.h> 
#include	<string.h>
#include	<termios.h>
#include	<unistd.h>
#include	<poll.h>

#define		BUFFER_LEN		4
#define		NUM_THREADS		2
#define		BIG_BUF			(3 * 4096)
#define         LINE_SEP                "\r\n"

pthread_t		threads[NUM_THREADS];
pthread_mutex_t		init_mutex = PTHREAD_MUTEX_INITIALIZER;
char			infilename[4096];
char 			*big_buf;

pthread_mutex_t signal_mutex;
pthread_cond_t signal_threshold_cv;

unsigned int ids[100][2] = {0};
unsigned int last_id = 0;
int last_signal = -1;

/* CO/TECH remote */
#define SIGNAL_RA1      0b101000010111010010101100
#define SIGNAL_RA0      0b101010101111011110111100

unsigned int signalRB1 = 0b101001110101001011110101;
unsigned int signalRB0 = 0b101001000100010111010101;

unsigned int signalRC1 = 0b101011000000111110001110;
unsigned int signalRC0 = 0b101010111001110101101110;

unsigned int signalRD1 = 0b101001000100010111010111;
unsigned int signalRD0 = 0b101001110101001011110111;


unsigned long signals[] = {
        // A1
        SIGNAL_RA1,
        // A0
        SIGNAL_RA0,
        // unsigned long signalRB1
        0b101001110101001011110101,
        // unsigned long signalRB0
        0b101001000100010111010101,
        // unsigned long signalRC1
        0b101011000000111110001110,
        // unsigned long signalRC0
        0b101010111001110101101110,

        // unsigned long signalRD1
        0b101001000100010111010111,
        // unsigned long signalRD0
        0b101001110101001011110111};


static void int_handler(int not_used)
{
	pthread_cancel(threads[0]);
	pthread_cancel(threads[1]);
	exit(1);
}

void add_id(unsigned int id)
{
    int found_id = -1;
    /* Check if the signal exists in the list of known signals */
    for (int i = 0; i < sizeof(signals) / sizeof(unsigned long); i++) {
        if (signals[i] == id) {
            found_id = i;
            break;
        }
    }
    if (found_id == -1)
        return;
    /* If it was seen already in this sequence then +1 */
    for (int i = 0; i < last_id; i++)
        if(ids[i][0] == id) {
            ids[i][1]++;
            return;
        }
    /* First time seen signal in this sequence */
    last_id++;
    ids[last_id + 1][0] = id;
    ids[last_id + 1][1] = 1;
}

/*
 * Main decode sequence. Takes the input data from big_buf, splits
 * it in lines and for every 2 lines it decodes:
 * if the high signal lasts longer than the low signal then a "1" was encoded
 * if the low signal lasts longer than the high signal then a "0" was encoded
 * 
 * TODO: there are sync signals for sure that are not taken into account
 * NOTE: the sequence must start with a 1
 * NOTE: long press on remote control will generate repeating values
 */
int decode_data()
{
    unsigned long time1, time2;
    unsigned int level1, level2;
    unsigned int id = 0;
    ssize_t read;
    unsigned int l = 0, b = 0;
    char *line1 = NULL, *line2 = NULL;
    size_t len = 0;
    char *buf = strdup((const char *)big_buf);
    memset(ids, 0, sizeof(ids));
    last_id = 0;
    printf("Processing: %s\n", big_buf);
    line1 = strtok(buf, LINE_SEP);
    while (line1) {
	line2 = strtok(NULL, LINE_SEP);
        if (!line2) {
            printf("\nNo even lines left!\n");
            break;
        }
        if (sscanf(line1, "%lu %u", &time1, &level1) != EOF &&
            sscanf(line2, "%lu %u", &time2, &level2) != EOF) {
            if (level1 == level2) {
                printf("\nWrong level sequence at line %u!\n", l);
                printf("\n%s\n%s\n", line1, line2);
                break;
            }
            
            if (time2 > 2000) {
                b = 0;
                id = 0;
                printf("\n");
                l += 2;
               	line1 = strtok(NULL, LINE_SEP);
                continue;
            }
            //printf("Got %lu usecs for level %u\n", time1, level1);
            id <<= 1;
            if (time1 > time2) {
                printf("1");
                id += 1;
            }
            else {
                printf("0");
            }
            b++;
            /* We expect 24 bits of data, when found then
             * start from the beggning */
            if (b == 24) {
                b = 0;
                printf(" %08X\n", id);
                add_id(id);
                printf("24 bits at %u\n", l + 2);
                /* Skip sync signals, one high, one low */
                for (int i = 0; i < 2; i++) {
		    line1 = strtok(NULL, LINE_SEP);
                    l++;
                }
                id = 0;

            }
        }
        l += 2;
	line1 = strtok(NULL, LINE_SEP);
    }
    free(buf);
    /* Nothing was found, no know signal was decoded */
    if(!last_id)
        return -1;
    /* Find the signal with the highest appearance rate */
    int max_id = 0;
    for(int i = 1; i < last_id; i++) {
        if (ids[i][1] > ids[max_id][1]) {
            max_id = i;
        }
    }
    return ids[max_id][0];
}

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf(stderr, "error %d from tcgetattr\n", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        //tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                fprintf(stderr, "error %d from tcsetattr\n", errno);
                return -1;
        }
        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf(stderr, "error %d from tggetattr\n", errno);
                return;
        }
	/* Break buffer into lines */
	tty.c_lflag |= ICANON;
        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                fprintf(stderr, "error %d setting term attributes", errno);
}


void *read_data(void/*jack_ringbuffer_t *rb*/)
{
	fd_set set;
	struct timeval timeout;
	struct pollfd pfd = {0,0,0};
	int rv, start = 0;
	int fd = open(infilename, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		fprintf(stderr, "error %d opening %s: %s", errno, infilename, strerror (errno));
		return NULL;
	}

	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 1);                // set blocking

 	FD_ZERO(&set); /* clear the set */
	FD_SET(fd, &set); /* add our file descriptor to the set */


	while(1) {
		int n = 0;
		char buf[100] = {0};
		if (start) {
			/* Wait 1000 miliseconds for an POLLIN event on STDIN. */
			pfd.fd = fd;
			pfd.events = POLLIN;
			rv = poll(&pfd, 1, 1000);
			if(rv == -1) {
				perror("select"); /* an error accured */
				exit(1);
			} else if(rv == 0) {
				printf("timeout\n"); /* a timeout occured */
                                last_signal = decode_data();
                                pthread_cond_signal(&signal_threshold_cv);
				start = 0;
				continue;
			}
			else {
				n = read(fd, buf, sizeof(buf) ); /* there was data to read */
			}
		}
		else {
			big_buf[0] = 0;
			memset(big_buf, 0, BIG_BUF);
                        pthread_mutex_unlock(&signal_mutex);
			n = read(fd, buf, sizeof(buf)); /* there was data to read */
                        pthread_mutex_lock(&signal_mutex);
                        last_signal = -1;
			start = 1;
			printf("New data\n");
		}
		strncat(big_buf, buf, n);
		//printf("GOT %d bytes: \n%s\n", n, buf);
	}
	pthread_exit(NULL);
}

void *play_data(void/*jack_ringbuffer_t *rb*/)
{
	int	rc;
        int     signal = -1;
	
	/*if(pthread_mutex_lock(&init_mutex) < 0){
		fprintf(stderr, "Failed to lock mutex!");
		exit(1);
	}*/
	
        pthread_mutex_lock(&signal_mutex);
        while (1) {
            pthread_cond_wait(&signal_threshold_cv, &signal_mutex);
            printf("Received SIGNAL: %06X\n", last_signal);
            switch(last_signal) {
                case SIGNAL_RA0:
                    printf("ALL OFF\n");
                    system("./lights_off.sh");
                    break;
                case SIGNAL_RA1:
                    printf("ALL ON\n");
                    system("./lights_on.sh");
                    break;
                default:
                    printf("Nothing\n");
            }
        }
        pthread_mutex_unlock(&signal_mutex);
	pthread_exit(NULL);
}

static const char *help="usage: %s [filename] [channel]\n";

int main(int argc, char **argv)
{
	unsigned int	i, rc, t;
	void			*func[NUM_THREADS] = {read_data, play_data};

	signal(SIGINT, int_handler);

	if(argc > 1){
		if(argc >= 2){
			strcpy(infilename, argv[1]);
		}
		else{
			printf(help, argv[0]);
			exit(1);
		}
		
	}
	else{
		strcpy(infilename, "/dev/ttyACM0");
	}	

	fprintf(stdout, "Playing %s\n", infilename);
	big_buf = malloc(BIG_BUF);
	if (!big_buf) {
		fprintf(stderr, "Failed to allocate big buf\n");
		exit(1);
	}
		

	if(pthread_mutex_lock(&init_mutex) < 0){
		fprintf(stderr, "failed to init mutex\n");
		exit(1);
	}
        pthread_mutex_init(&signal_mutex, NULL);
        pthread_cond_init (&signal_threshold_cv, NULL);

	for(t=0; t < NUM_THREADS; t++){
		fprintf(stdout, "creating thread %d\n", t);
		rc = pthread_create(&threads[t], NULL, func[t], NULL/*(void *)rb*/);
		if (rc){
			fprintf(stderr, "ERROR; return code from pthread_create() is %d\n", rc);
			exit(-1);
		}
	}
	
	
	pthread_join(threads[0], (void *) &rc);
	pthread_cancel(threads[1]);
        
        pthread_mutex_destroy(&signal_mutex);
        pthread_cond_destroy(&signal_threshold_cv);

	return 0 ;
}
