#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
// #include <asm/termios.h>
#include <alsa/asoundlib.h>
// #include <dispatch/dispatch.h>

int open_port(int *serial_port) {
    const char *port = "/dev/ttyACM";

    char port_name[128];
    int i;
    for (i = 0; i < 10; ++i) {
        sprintf(port_name, "%s%d", port, i);
        if (access(port_name, F_OK) == 0) {
            break;
        }

        if (i == 9) {
            printf("Error: No serial port found\n");
            return 1;
        }
    }

    *serial_port = open(port_name, O_RDWR);

    printf("a\n");
    // Check for errors
    if (*serial_port < 0) {
        printf("Error %i from open serial port: %s\n", errno, strerror(errno));
        return 1;
    }

    // Configure the serial port
    struct termios tty;
    if(tcgetattr(*serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    printf("b\n");
    tty.c_cflag &= ~PARENB; // Disable parity
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE; // Clear size bits
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ and ignore control lines

    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT, SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // tty.c_cc[VTIME] = 1; // Wait for up to 1s, returning as soon as any data is received.
    // tty.c_cc[VMIN] = 0;

    // tty.c_cflag &= ~CBAUD;
    // tty.c_cflag |= BOTHER;
    // tty.c_ispeed = 31250;
    // tty.c_ospeed = 31250;
    // Set baud rate

    // cfsetispeed(&tty, B19200);
    cfsetispeed(&tty, B38400);
    // cfsetispeed(&tty, 31250);
    // cfsetospeed(&tty, 31250);
    // cfsetispeed(&tty, B115200);
    // cfsetospeed(&tty, B115200);
    // 
    // cfsetospeed(&tty, B9600);
    // cfsetispeed(&tty, 31250);
    // cfsetospeed(&tty, 31250);
    
 
    // struct termios2 tio;
    // ioctl(*serial_port, TCGETS2, &tio);
    // tio.c_cflag &= ~CBAUD;
    // tio.c_cflag |= BOTHER;
    // tio.c_ispeed = 31250;
    // tio.c_ospeed = 31250;
    /* do other miscellaneous setup options with the flags here */
    // Save tty settings
    // if (ioctl(*serial_port, TCSETS2, &tty) != 0) {
    //     printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    //     return 1;
    // }


    // Save tty settings
    if (tcsetattr(*serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }
    printf("c\n");
    
    return 0;
}

int read_serial(int serial_port, int *read_buf) {
    // Read bytes
    int num_bytes = read(serial_port, read_buf, sizeof(read_buf));

    // Check for errors
    if (num_bytes < 0) {
        printf("Error reading: %s\n", strerror(errno));
        return 1;
    }

  // Close the serial port
  close(serial_port);
  return 0;
}

#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>
#include <unistd.h>

int main() {
  fprintf(stdout, "# sudo modprobe snd-virmidi\n");
  system("sudo modprobe snd-virmidi\n");
  // fprintf(stdout, "# sudo socat -d -d pty,raw,echo=0,link=/dev/ttyACM0 pty,raw,echo=0\n");
  // system("sudo socat -d -d pty,raw,echo=0,link=/dev/ttyACM0 pty,raw,echo=0\n");

  fprintf(stdout, "stard started!\n");

  snd_rawmidi_t* midi_out;
  int status;
  unsigned char note_on[3] = {0x90, 60, 116};  // Note On: Channel 1, Note C4, Velocity 86
  unsigned char note_off[3] = {0x90, 60, 0};  // Note Off: Channel 1, Note C4, Velocity 0

  // Open the serial port
  int serial_port, res;
  res = open_port(&serial_port);
  if (res != 0) {
      fprintf(stderr, "Error opening the serial port\n");
      system("sudo modprobe -r snd-virmidi\n");
      return res;
  }
  printf("Opened serial port\n");

  // Open the MIDI output interface
  status = snd_rawmidi_open(NULL, &midi_out, "hw:2,1", SND_RAWMIDI_SYNC);
  if (status < 0) {
      fprintf(stderr, "Error opening MIDI output: %s\n", snd_strerror(status));
      system("sudo modprobe -r snd-virmidi\n");
      exit(1);
  }
  printf("Opened MIDI output\n");

  int noteon[256];
  int notewason[256];
  memset(&noteon, 0, sizeof(noteon));
  memset(&notewason, 0, sizeof(notewason));

  unsigned char note_buff[128];
  int off = 0, num_bytes;
  // while (true) {
  //   num_bytes = read(serial_port, &note_buff[off], sizeof(note_buff));
  // }
  printf("beginning read loop\n");

  for (int i = 0; i < 800; ++i) {
    // Read bytes
    num_bytes = read(serial_port, &note_buff[off], sizeof(note_buff));
    // printf("Read %i bytes:\n", num_bytes);

    if (num_bytes == 0) {
      --i;
      continue;
    }

    // if(i % 40 == 0) {
    //   printf("i: %d\n", i);
    //   for (int j = 0; j < 128; ++j) {
    //     if (noteon[j] != -1 && noteon[j] < i - 40) {
    //       printf("Forcing Note Off %02X\n", j);
    //       noteon[j] = -1;
    //       note_off[1] = j;
    //       snd_rawmidi_write(midi_out, note_off, 3);
    //       snd_rawmidi_drain(midi_out);
    //     }
    //   }
    // }
    
    // Output read bytes
    printf("Read %i bytes:\n", num_bytes);
    for (int j = 0; j < num_bytes; j++) {
        printf("  %02X", (__uint8_t)note_buff[j]); // %02X
    }
    printf("\n");
    int j = 0;
    for (; j < off + num_bytes;) {
      if (note_buff[j] != 0x90) {
        printf("Format Error!!! Not Note On!!  %02x\n", note_buff[off + j]);
        ++j;
        continue;
      }
      if (j + 2 >= off + num_bytes) {
        break;
      }

      int note = note_buff[j + 1];
      int vel = note_buff[j + 2];
      // printf("Note %02X %d\n", note, vel);

      if (vel == 0) {
        // Note is being turned off
        if (noteon[note] == 0) {
          // Note was previously off
          printf("Receive Error!!! Note Off without Note On!!  %02x %d\n", note, noteon[note]);
        }
        noteon[note] = 0;
      } else {
        // Note is being turned on
        if (noteon[note] != 0) {
          // Note was previously on
          printf("Receive Error!!! Note On without Note Off!!  %02x %d\n", note, noteon[note]);
        }
        noteon[note] = 1;
        notewason[note] += 1;
      }

      // printf("Note On\n");
      // Send the Note On message
      // printf("send %02X %02X %02X\n", note_buff[j], note, vel);
      snd_rawmidi_write(midi_out, &note_buff[j], 3);
      snd_rawmidi_drain(midi_out);

      note_buff[j + 0] = 255;
      note_buff[j + 1] = 255;
      note_buff[j + 2] = 255;
      j += 3;

      // printf("\n");
    }
    off = (off + num_bytes - j) % 3;
    for (int k = 0; k < off; ++k) {
      note_buff[k] = note_buff[j + k];
    }
    
    // // Send the Note Off message
    // snd_rawmidi_write(midi_out, note_off, 3);
    // snd_rawmidi_drain(midi_out);

    // Wait for 2 seconds before the next note
    // sleep(1);
  }

  for (int j = 0; j < 128; ++j) {
    if (notewason[j] != 0) {
      printf("Note %02X was on %d times\n", j, notewason[j]);
    }
  }

  // Close the serial port
  close(serial_port);

  // Close the MIDI output
  snd_rawmidi_close(midi_out);

  system("sudo modprobe -r snd-virmidi\n");

  return 0;
}
