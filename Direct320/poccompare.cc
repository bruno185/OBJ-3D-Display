/*
 * POC comparaison vitesse QuickDraw vs accès direct SHR 320x200
 *
 * - Dessine un polygone avec QuickDraw, mesure le temps
 * - Efface l'écran
 * - Dessine le même polygone avec draw320.c, mesure le temps
 * - Affiche les deux temps
 *
 * Apple IIGS, ORCA/C, 2025
 */
#include <stdio.h>
#include <misctool.h>   // Pour GetTick()
#include <memory.h>
#include <quickdraw.h>  // Pour QuickDraw

#include "draw320.c"   // Routines d'accès direct
#include "asm.h"        // my assembler functions

// Tableau de polygones complexes à tester
typedef struct { const char* name; int n; Point2D* pts; } PolyTest;

Point2D poly_star5[10] = {
    {100, 60}, {112, 100}, {155, 100}, {120, 120}, {135, 160},
    {100, 135}, {65, 160}, {80, 120}, {45, 100}, {88, 100}
};
Point2D poly_fleche[7] = {
    {60, 60}, {120, 60}, {120, 100}, {160, 100}, {90, 160}, {20, 100}, {60, 100}
};
Point2D poly_S[8] = {
    {60, 60}, {120, 60}, {120, 100}, {80, 100}, {80, 140}, {140, 140}, {140, 180}, {60, 180}
};
Point2D poly_bowtie[4] = {
    {60, 60}, {140, 140}, {60, 140}, {140, 60}
};
Point2D poly_star8[8] = {
    {80, 60}, {120, 80}, {160, 60}, {140, 100}, {160, 140}, {120, 120}, {80, 140}, {100, 100}
};

#define NB_POLYS 5
PolyTest polys[NB_POLYS] = {
    {"Étoile 5 branches (auto-croisé)", 10, poly_star5},
    {"Flèche concave", 7, poly_fleche},
    {"S concave", 8, poly_S},
    {"Bowtie (auto-croisé)", 4, poly_bowtie},
    {"Étoile 8 sommets (concave)", 8, poly_star8}
};

typedef unsigned int Word;
typedef unsigned long LongWord;



// Efface l'écran SHR 320x200 avec une couleur 0-15 (rapide, précalcule l'octet)
void clearscreen320(uint8_t color) {
    uint8_t* vram = (uint8_t*)0xE12000;
    uint8_t fill = (color << 4) | color;
    for (int i = 0; i < 160*200; i++) vram[i] = fill;
}

// Définition explicite si absente
#ifndef Handle
typedef char **Handle;
#endif

typedef struct {
    int polySize;
    struct { int h1, v1, h2, v2; } polyBBox;
    Point polyPoints[16]; // Taille max d'un polygone à afficher
} DynamicPolygon;

void drawpolyquickdraw(Point2D* pts, int n, int fillcol, int bordercol) {
    int min_x = -1, max_x = -1, min_y = -1, max_y = -1;
    int size = 2 + 8 + n*4;
    Handle h = (Handle)NewHandle((long)size, userid(), 0xC015, 0L);
    if (!h) {
        MoveTo(10, 10);
        printf("Erreur : NewHandle a echoue pour le polygone QuickDraw (taille=%d)\n", size);
        return;
    }
    HLock(h);
    DynamicPolygon* poly = (DynamicPolygon*)*h;
    poly->polySize = size;
    // Initialiser la bounding box
    for (int i = 0; i < n; i++) {
        poly->polyPoints[i].h = pts[i].x;
        poly->polyPoints[i].v = pts[i].y;
        if (min_x == -1 || pts[i].x < min_x) min_x = pts[i].x;
        if (max_x == -1 || pts[i].x > max_x) max_x = pts[i].x;
        if (min_y == -1 || pts[i].y < min_y) min_y = pts[i].y;
        if (max_y == -1 || pts[i].y > max_y) max_y = pts[i].y;
    }
    poly->polyBBox.h1 = min_x;
    poly->polyBBox.v1 = min_y;
    poly->polyBBox.h2 = max_x;
    poly->polyBBox.v2 = max_y;
    SetSolidPenPat(fillcol);
    PaintPoly(h);
    SetSolidPenPat(bordercol);
    FramePoly(h);
    HUnlock(h);
    DisposeHandle(h);
}

int main() {
    startgraph(320);
    for (int p = 0; p < NB_POLYS; ++p) {
        // printf("\n==== Polygone %d/%d : %s ====\n", p+1, NB_POLYS, polys[p].name);
        
        clearscreen320(0);
        long t0 = GetTick();
        drawpolyquickdraw(polys[p].pts, polys[p].n, 7, 2);
        long t1 = GetTick();
        keypress();
        //clearscreen320(0);
        long t2 = GetTick();
        draw_filled_poly_with_border(polys[p].pts, polys[p].n, 14, 2);
        long t3 = GetTick();
        MoveTo(10, 10);printf("QuickDraw: %ld ticks\n", t1-t0);
        MoveTo(10, 20);printf("Direct320: %ld ticks\n", t3-t2);
        keypress();
    }

    endgraph();
    shroff();
    printf("\nTous les polygones ont été affichés.\n");
    return 0;
}

//#append "framepoly_asm.s";
#append "drawline320.asm"
