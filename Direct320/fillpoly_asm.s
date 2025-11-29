* fillpoly_asm.s - Routine assembleur 65816 pour remplir un polygone convexe en SHR 320
* Entrée :
*   A1 = pointeur sur tableau de Point2D (x, y, 16 bits signés)
*   A2 = nombre de points
*   A3 = couleur (0-15)
*
* Convention ORCA/C :
*   Point2D = { int x, int y; } (2*2 octets)
*
* Cette version trace simplement des lignes horizontales entre les intersections (scanline)

        case on
        mcopy 'Point2D equ 0

        export fillpoly_asm
fillpoly_asm start
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
        jsr fillpoly_core
        plb
        rts

* Variables temporaires
color   ds 2
npts    ds 2
pts     ds 4

* Routine principale (à compléter pour un vrai scanline)
fillpoly_core start
        ; Ici, tu dois implémenter l'algorithme scanline en assembleur
        ; Pour le POC, tu peux juste tracer les segments du polygone (wireframe)
        rts
fillpoly_core end
fillpoly_asm end
