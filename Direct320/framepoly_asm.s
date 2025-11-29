* framepoly_asm.s - Routine assembleur 65816 pour tracer le contour d'un polygone en SHR 320
* Entrée :
*   A1 = pointeur sur tableau de Point2D (x, y, 16 bits signés)
*   A2 = nombre de points
*   A3 = couleur (0-15)
*
* Convention ORCA/C :
*   Point2D = { int x, int y; } (2*2 octets)
*
* Cette version doit tracer chaque segment du polygone

        case on

framepoly_asm start
Point2D equ 0
        phb
        phk
        plb
        lda 6,s        ; couleur (A3)
        sta >color
        lda 4,s        ; nombre de points (A2)
        sta >npts
        lda 2,s        ; pointeur sur points (A1)
        sta >pts
        lda 0,s
        sta >pts+2
        jsr framepoly_core
        plb
        rts

* Variables temporaires
color   ds 2
npts    ds 2
pts     ds 4

        end

* Routine principale (à compléter pour un vrai tracé)
framepoly_core start
        lda >npts
        and #$FFFF
*       
*
        cmp #1
        bcc done
        ldx #0              ; i = 0
loop_points anop
        ldy #0
        lda >pts
        sta >ptptr
        lda >pts+2
        sta >ptptr+2
        lda x
        asl         ; x * 2
        asl         ; x * 4
        tay         ; y = 4 * x
        lda [ptptr],y      ; x0 (low word)
        sta >x0
        * Routine 65816 : drawline320_asm
        * Entrée : x0, y0, x1, y1, color (toutes 16 bits, signés pour x/y)
        * Sortie : trace le segment (x0,y0)-(x1,y1) en SHR 320, couleur directe
        end
drawline320_asm start
                phb
                phk
                plb
                lda >x0
                sta >dx0
                lda >y0
                sta >dy0
                lda >x1
                sta >dx1
                lda >y1
                sta >dy1

                lda >dx1
                sec
                sbc >dx0
                sta >dx
                bpl dx_pos
                eor #$FFFF
                inc a
                sta >adx
                lda #$FFFF
                sta >sx
                bra dx_done
dx_pos:
                sta >adx
                lda #1
                sta >sx
dx_done:

                lda >dy1
                sec
                sbc >dy0
                sta >dy
                bpl dy_pos
                eor #$FFFF
                inc a
                sta >ady
                lda #$FFFF
                sta >sy
                bra dy_done
dy_pos:
                sta >ady
                lda #1
                sta >sy
dy_done:

                lda >adx
                cmp >ady
                bpl major_x
                bra major_y

major_x:
                lda >adx
                lsr a
                sta >err
                lda >dy0
                sta >y
                lda >dx0
                sta >x
                ldy >adx
                iny
mx_loop:
                jsr plotpixel
                lda >err
                sec
                sbc >ady
                sta >err
                bpl mx_noinc
                lda >y
                clc
                adc >sy
                sta >y
                lda >err
                clc
                adc >adx
                sta >err
mx_noinc:
                lda >x
                clc
                adc >sx
                sta >x
                dey
                bne mx_loop
                jsr plotpixel
                plb
                rts

major_y:
                lda >ady
                lsr a
                sta >err
                lda >dx0
                sta >x
                lda >dy0
                sta >y
                ldy >ady
                iny
my_loop:
                jsr plotpixel
                lda >err
                sec
                sbc >adx
                sta >err
                bpl my_noinc
                lda >x
                clc
                adc >sx
                sta >x
                lda >err
                clc
                adc >ady
                sta >err
my_noinc:
                lda >y
                clc
                adc >sy
                sta >y
                dey
                bne my_loop
                jsr plotpixel
                plb
                rts

* Routine de tracé d'un pixel en SHR 320 (x, y, color)
plotpixel:
                lda >y
                cmp #0
                bmi plotret
                cmp #200
                bpl plotret
                lda >x
                cmp #0
                bmi plotret
                cmp #319
                bpl plotret
* Calcul adresse VRAM SHR 320: $E12000 + y*160 + (x>>1)
                lda >y
                sta >tmp16
                asl a
                asl a
                clc
                adc >tmp16
                asl a
                asl a
                asl a
                asl a
                sta >tmp16
                lda >x
                lsr a
                clc
                adc >tmp16
                tax
                lda >y
                lsr a
                lsr a
                lsr a
                lsr a
                lsr a
                lsr a
                lsr a
                lsr a
                sta >tmp8
                lda #$E1
                xba
                lda #$2000
                clc
                adc >tmp16
                sta >tmp16
                txa
                clc
                adc >tmp16
                tax
                * Lecture-modification-écriture du mot SHR
                lda $E12000,x
                ldy >x
                tya
                and #1
                beq even
                lda >color
                asl a
                asl a
                asl a
                asl a
                ora $E12000,x
                sta $E12000,x
                bra plotret
even:
                lda >color
                ora $E12000,x
                sta $E12000,x
plotret:
                rts

        * Variables temporaires pour drawline
        dx      ds 2
        dy      ds 2
        adx     ds 2
        ady     ds 2
        sx      ds 2
        sy      ds 2
        err     ds 2
        x       ds 2
        y       ds 2
        dx0     ds 2
        dy0     ds 2
        dx1     ds 2
        dy1     ds 2
        tmp16   ds 2
        tmp8    ds 2
        iny
        lda [ptptr],y      ; y0
        sta >y0
        inx
        cpx >npts
        bne not_last
        ldx #0             ; boucler sur le premier point
not_last:
        lda #4
        mul x
        tay
        lda [ptptr],y      ; x1
        sta >x1
        iny
        iny
        lda [ptptr],y      ; y1
        sta >y1
        ; Appel à la routine de tracé de segment (à écrire ou à lier)
        jsr drawline320_asm
        inx
        cpx >npts
        blt loop_points
        bra done

done:   rts

* Variables temporaires
x0      ds 2
y0      ds 2
x1      ds 2
y1      ds 2
ptptr   ds 4
framepoly_core end
framepoly_asm end
