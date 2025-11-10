#include <stdio.h>
#include <asm.h>
#include <string.h>
#include <misctool.h>
#include <stdlib.h>
#include <math.h>
#include <quickdraw.h>
#include <event.h>
#include <memory.h>
#include <window.h>
#include <orca.h>


int main() {
        int i;

        i= 0;
        startgraph(320);                       /* initialize QuickDraw */
        keypress();                             /* wait for a keypress */
        endgraph();                            /* close QuickDraw */
        keypress();
   
        return 0;
}