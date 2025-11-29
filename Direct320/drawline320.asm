* drawline320_asm.s - Routine assembleur 65816 pour tracer une ligne en SHR 320x200
* Entrée :
*   x0, y0, x1, y1 : coordonnées (16 bits signés, C calling convention)
*   color : couleur (0-15, 8 bits)
*
* Convention ORCA/C :
*   int drawline320_asm(int x0, int y0, int x1, int y1, uint8_t color);
*
* Cette version utilise l'algorithme de Bresenham entier

        case on

drawline320_asm start
* Dépile l'adresse de retour 16 bits (low, puis high)
        pla         ; low byte
        sta retaddr
        pla 
        sta retaddr+2

* Dépile les paramètres (5*2 octets)
        pla         ; y0
        sta y0
        pla         ; x0
        sta x0
        pla         ; y1
        sta y1
        pla         ; x1
        sta x1
        pla         ; color
        sta color

        lda retaddr+2
        pha
        lda retaddr
        pha
        rtl


* Algorithme de Bresenham (entier, toutes directions)
* Version simple : appel C à put_pixel320 (x, y, color)

* Charger les variables
        lda x0
        sta curx
        lda y0
        sta cury

        lda x1
        sta endx
        lda y1
        sta endy

        lda color
        sta curcolor

* dx = abs(x1-x0)
        lda x1
        sec
        sbc x0
        bpl dx_pos
        eor #$FFFF
        inc a
dx_pos anop
        sta dx

* sx = (x0 < x1) ? 1 : -1
        lda x0
        cmp x1
        bcc sx_pos
        lda #-1
        bra sx_set
sx_pos anop
        lda #1
sx_set anop
        sta sx

* dy = -abs(y1-y0)
        lda y1
        sec
        sbc y0
        bpl dy_pos
        eor #$FFFF
        inc a
dy_pos anop
        sta tmp
        lda #0
        sec
        sbc tmp
        sta dy

* sy = (y0 < y1) ? 1 : -1
        lda y0
        cmp y1
        bcc sy_pos
        lda #-1
        bra sy_set
sy_pos anop
        lda #1
sy_set anop
        sta sy

* err = dx + dy
        lda dx
        clc
        adc dy
        sta err

main_loop anop
* put_pixel320(curx, cury, color)
        lda curx
        pha
        lda cury
        pha
        lda curcolor
        pha
        jsl put_pixel320
        pla
        pla
        pla

        lda curx
        cmp endx
        bne not_end
        lda cury
        cmp endy
        beq done
not_end anop

        lda err
        asl a
        sta e2

        lda e2
        cmp dy
        blt skip_x
        lda err
        clc
        adc dy
        sta err
        lda curx
        clc
        adc sx
        sta curx
skip_x anop

        lda e2
        cmp dx
        beq eg
        bcs skip_y
eg     anop
        lda err
        clc
        adc dx
        sta err
        lda cury
        clc
        adc sy
        sta cury
skip_y anop

        bra main_loop

done anop
* Réempile l'adresse de retour 16 bits (high, puis low)
        lda retaddr
        pha
        lda retaddr+2
        pha
        rtl

* Variables temporaires

x0      ds 2
y0      ds 2
x1      ds 2
y1      ds 2
color   ds 2
retaddr ds 4


curx    ds 2
cury    ds 2
endx    ds 2
endy    ds 2
curcolor ds 2
dx      ds 2
dy      ds 2
sx      ds 2
sy      ds 2
err     ds 2
e2      ds 2
tmp     ds 2


        end
