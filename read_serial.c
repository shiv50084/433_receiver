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

pthread_t		threads[NUM_THREADS];
pthread_mutex_t		init_mutex = PTHREAD_MUTEX_INITIALIZER;
char			infilename[PATH_MAX];
char 			*big_buf;

unsigned int ids[100][2] = {0};
unsigned int last_id = 0;

/* CO/TECH remote */
unsigned long signalRA1 = 0b101000010111010010101100;
unsigned long signalRA0 = 0b101010101111011110111100;

unsigned long signalRB1 = 0b101001110101001011110101;
unsigned long signalRB0 = 0b101001000100010111010101;

unsigned long signalRC1 = 0b101011000000111110001110;
unsigned long signalRC0 = 0b101010111001110101101110;

unsigned long signalRD1 = 0b101001000100010111010111;
unsigned long signalRD0 = 0b101001110101001011110111;


unsigned long signals[] = {
        // A1
        0b101000010111010010101100,
        // A0
        0b101010101111011110111100,
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
    for (int i = 0; i < sizeof(signals) / sizeof(unsigned long); i++) {
        if (signals[i] == id) {
            found_id = i;
            break;
        }
    }
    if (found_id == -1)
        return;
    for (int i = 0; i < last_id; i++)
        if(ids[i][0] == id) {
            ids[i][1]++;
            return;
        }
    last_id++;
    ids[last_id + 1][0] = id;
    ids[last_id + 1][1] = 1;
}

int decode_data()
{
    unsigned long time1, time2;
    unsigned int level1, level2;
    unsigned int id = 0;
    ssize_t read;
    unsigned int l = 0, b = 0;
    char *line1 = NULL, *line2 = NULL;
    size_t len = 0;
    char *buf = strdup(big_buf);
    memset(ids, 0, sizeof(ids));
    last_id = 0;
    printf("Processing: %s\n", big_buf);
    line1 = strtok(buf, "\r\n");
    while (line1) {
	line2 = strtok(NULL, "\r\n");
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
               	line1 = strtok(NULL, "\r\n");
                continue;
            }
            //printf("Got %lu usecs for level %u\n", time1, level1);
            if (time1 > time2) {
                printf("1");
                id = (id << 1) + 1;
            }
            else {
                printf("0");
                id = id << 1;
            }
            b++;
            if (b == 24) {
                b = 0;
                printf(" %08X\n", id);
                add_id(id);
                printf("24 bits at %u\n", l + 2);
                for (int i = 0; i < 2; i++) {
		    line1 = strtok(NULL, "\r\n");
                    l++;
                }
                id = 0;

            }
        }
        l += 2;
	line1 = strtok(NULL, "\r\n");
    }
    free(buf);
    if(!last_id)
        return -1;
    int max_id = 0;
    for(int i = 0; i < last_id; i++) {
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
        tty.c_cflag &= ~CRTSCTS;

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
	//tty.c_lflag |= ICANON;
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
			/* Wait 5000 miliseconds for an POLLIN event on STDIN. */
			pfd.fd = fd;
			pfd.events = POLLIN;
			rv = poll(&pfd, 1, 1000);
#if 0
			timeout.tv_sec = 2;
			timeout.tv_usec = 0;
			rv = select(fd + 1, &set, NULL, NULL, &timeout);
#endif
			if(rv == -1) {
				perror("select"); /* an error accured */
				return NULL;
			} else if(rv == 0) {
				printf("timeout\n"); /* a timeout occured */
				printf("FINAL ID: %06X\n", decode_data());
				start = 0;
				continue;
			}
			else {
				//set_blocking (fd, 0);                // set non blocking
				n = read(fd, buf, sizeof(buf) ); /* there was data to read */
			}
		}
		else {
			//set_blocking (fd, 1);                // set blocking
			big_buf[0] = 0;
			memset(big_buf, 0, BIG_BUF);
			n = read(fd, buf, sizeof(buf)); /* there was data to read */
			start = 1;
			printf("New data\n");
		}
		strncat(big_buf, buf, n);
		printf("GOT %d bytes: \n%s\n", n, buf);
	}
#if 0
	SNDFILE				*infile;
	SF_INFO				sfinfo;
	int					readcount, rb_readcount;
	snd_pcm_format_t	pcm_format;
	int					m;
	codec_filter_t		filter;
	
	char	*tmp,*filter_buff;
	uint32_t *pt;
	int		*buffer;
	int		ret, res;
	int		buf_size, i, j, neg;
	int		format;
	
	if ( !(infile = sf_open(infilename, SFM_READ, &sfinfo) ) )
	{
		/* Open failed so print an error message. */
		LOG_ERROR("Not able to open input file %s!", infilename ) ;
		/* Print the error message from libsndfile. */
		LOG_ERROR(sf_strerror(NULL));
		pthread_cancel(threads[1]);
		pthread_exit(NULL);
	}

	if ( sfinfo.channels > MAX_CHANNELS )
	{
		LOG_ERROR("Found %d channels, not able to process more than %d channels",
			 sfinfo.channels, MAX_CHANNELS ) ;
		pthread_cancel(threads[1]);
		pthread_exit(NULL);
	}
	
	if( channel >= sfinfo.channels)
	{
		LOG_ERROR("Invalid channel selected, wanted %d but %d are available",
				  channel + 1, sfinfo.channels);
		pthread_cancel(threads[1]);
		pthread_exit(NULL);
	}

	format = sfinfo.format & 0xFF;
	if(format == SF_FORMAT_PCM_16){
		pcm_format = SND_PCM_FORMAT_S16_LE;
		m = 2;
		filter = codec_l16_filter;
	}
	else if(format == SF_FORMAT_PCM_24 ||
		format == SF_FORMAT_PCM_32){
		pcm_format = SND_PCM_FORMAT_S32_LE;
		m = 4;
		filter = codec_l32_filter;
	}
	else{
		LOG_ERROR("Invalid PCM format in payload!");
		pthread_cancel(threads[1]);
		pthread_exit(NULL);
	}

	if( !(snd_buf_size = snd_set_params(snd_handle, &snd_frames_no,
		pcm_format, sfinfo.samplerate, 2)) ){
		LOG_ERROR("Failed to set hw parameters!");
		pthread_cancel(threads[1]);
		pthread_exit(NULL);
	}
	if(pthread_mutex_unlock(&init_mutex) < 0){
		LOG_ERROR("Failed to unlock mutex!");
		pthread_cancel(threads[1]);
		pthread_exit(NULL);
	}

	buf_size = 2048;
	buffer = malloc(buf_size * sizeof(int));
	tmp = malloc(m * buf_size);
	filter_buff = malloc(buf_size * sizeof(int));

restart:
	while ( (readcount = sf_read_int(infile, buffer, buf_size))>0 )
	{
		pthread_testcancel();
		pt = tmp;

		for(i = 0; i < readcount; i++){
			res = buffer[i];
			*pt = widi_fec_encode(res >> 8);
			pt++;
		}

		j = filter(tmp, readcount * m, sfinfo.channels, channel, filter_buff);
		pt = filter_buff;
		widi_fec_decode_buf(pt, j);
		while( (ret = jack_ringbuffer_write(rb, pt, j)) != j){
			if(ret == -1)
				break;
			else if(ret != j){
				j -= ret;
				pt += ret;
			}
		}
	}
	printf("thread finished\n");
	free(filter_buff);
	free(buffer);
	free(tmp);
	sf_close(infile);
#endif
	pthread_exit(NULL);
}

void *play_data(void/*jack_ringbuffer_t *rb*/)
{
#if 0
	uint8_t	*buffer;
	int	rc;
	size_t	readcount;
	
	if(pthread_mutex_lock(&init_mutex) < 0){
		LOG_ERROR("Failed to lock mutex!");
		exit(1);
	}

	/* don't start until the stream is setup correctly */
	if(snd_pcm_state(snd_handle) != SND_PCM_STATE_PREPARED || !snd_buf_size){
		LOG_ERROR("Initialization failed, cannot play sound file!");
		exit(1);
	}

	buffer = malloc(snd_buf_size);
	while ( (readcount = jack_ringbuffer_read(rb, buffer, snd_buf_size)) == readcount)
	{
		pthread_testcancel();
		if(readcount != snd_buf_size){
			LOG_ERROR("readcount(%u) != snd_buf_size(%u)", readcount,
				  snd_buf_size);
			break;
		}

		rc = snd_pcm_writei(snd_handle, buffer, snd_frames_no);
		if (rc == -EPIPE) {
			/* EPIPE means underrun */
			LOG_WARNING("underrun occurred");
			snd_pcm_prepare(snd_handle);
		} else if (rc < 0) {
			LOG_ERROR("error from writei: %s", snd_strerror(rc));
			break;
		}  else if (rc != (int)snd_frames_no) {
			LOG_WARNING("short write, write %d frames", rc);
		}
	}
	free(buffer);
#endif
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

	return 0 ;
}
