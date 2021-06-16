#define CLI_DES_POS_INCR 0.01


/**
 Linux (POSIX) implementation of _kbhit().
 Morgan McGuire, morgan@cs.brown.edu
 */
#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
// #include <stropts.h>
#include <sys/ioctl.h>

int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}


//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                                                          //
//                         Non-blocking CLI                                 //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
/*
// Thank you Stack Overflow!
// https://stackoverflow.com/questions/448944/c-non-blocking-keyboard-input

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>




struct termios orig_termios;

void reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}


void set_conio_terminal_mode()
{
    struct termios new_termios;

    // take two copies - one for now, one for later 
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    // register cleanup handler, and set the new terminal mode 
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}

int kbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

int getch()
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}
*/

int updateState(float &x_d, float &y_d, float &z_d, double &r_d, double &p_d, double &yaw_d) {
    if (_kbhit()) {
        switch (getchar()) {
            case 'x': 
                printf("\n\rExiting!\n\r");
                //reset_terminal_mode();
                return 1;
                break;

            // CLI for posiion setpoint (testing follower mode)
            case 'q':
                r_d += CLI_DES_POS_INCR;
                break;
            case 'e':
                r_d -= CLI_DES_POS_INCR;
                break;
            case 'r':
                p_d += CLI_DES_POS_INCR;
                break;
            case 'f':
                p_d -= CLI_DES_POS_INCR;
                break;
            case 'w':
                x_d += CLI_DES_POS_INCR;
                break;
            case 's':
                x_d -= CLI_DES_POS_INCR;
                break;
            case 'd':
                y_d += CLI_DES_POS_INCR;
                break;
            case 'a':
                y_d -= CLI_DES_POS_INCR;
                break;
            case 't':
                z_d -= CLI_DES_POS_INCR;
                break;
            case 'g':
                z_d += CLI_DES_POS_INCR;
                break;
            case 'z':
                yaw_d += CLI_DES_POS_INCR;
                break;
            case 'c':
                yaw_d += CLI_DES_POS_INCR;
                break;
            case 'o':
                x_d = 0.0;
                y_d = 0.0;
                z_d = 0.0;
                r_d = 0.0;
                p_d = 0.0;
                yaw_d = 0.0;
                break;

            default :
                break;
                //printf("Input = %c \n\r",inputChar);
        }
    }

    return 0;
}