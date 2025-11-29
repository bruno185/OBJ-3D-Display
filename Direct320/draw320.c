/*
 * Routines d'affichage direct SHR 320x200 Apple IIGS
 * --------------------------------------------------
 * - Placement pixel (put_pixel320)
 * - Tracé de ligne (draw_line320, algorithme de Bresenham)
 * - Remplissage de polygone (fill_polygon320, algorithme scanline robuste)
 *
 * Ces routines permettent d'afficher des graphismes en mode 320x200 16 couleurs
 * en accédant directement à la mémoire vidéo (VRAM) de l'Apple IIGS.
 *
 * Version C pur, 2025
 */
#include <stdint.h>

// Adresse de base de la mémoire vidéo SHR 320x200
#define VRAM_BASE ((uint8_t*)0xE12000)
// Largeur et hauteur de l'écran en pixels
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 200

// -----------------------------------------------------------------------------
// Place un pixel à la position (x, y) avec la couleur 0-15 en mode SHR 320x200.
// Chaque octet VRAM contient 2 pixels (4 bits chacun).
// x impair : bits 3-0, x pair : bits 7-4
// -----------------------------------------------------------------------------
void put_pixel320(int x, int y, uint8_t color) {
    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) return; // Hors écran
    uint8_t* vram = VRAM_BASE + (y * 160) + (x >> 1); // 160 octets/ligne
    if (x & 1)
        *vram = (*vram & 0xF0) | (color & 0x0F); // pixel impair (droite)
    else
        *vram = (*vram & 0x0F) | ((color & 0x0F) << 4); // pixel pair (gauche)
}

// -----------------------------------------------------------------------------
// Trace une ligne entre (x0, y0) et (x1, y1) avec la couleur donnée
// Utilise l'algorithme de Bresenham (entier, rapide, toutes directions)
// -----------------------------------------------------------------------------
void draw_line320(int x0, int y0, int x1, int y1, uint8_t color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    while (1) {
        put_pixel320(x0, y0, color); // Place le pixel courant
        if (x0 == x1 && y0 == y1) break; // Fin
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; } // Avance en x
        if (e2 <= dx) { err += dx; y0 += sy; } // Avance en y
    }
}

// -----------------------------------------------------------------------------
// Structure pour représenter un point 2D (coordonnées entières)
// -----------------------------------------------------------------------------
typedef struct { int x, y; } Point2D;

// -----------------------------------------------------------------------------
// Remplissage d'un polygone (convexe, concave ou auto-croisé) par scanline
// - pts : tableau de sommets (x, y)
// - n   : nombre de sommets
// - color : couleur à utiliser (0-15)
//
// Algorithme :
// 1. Recherche le min et max Y du polygone
// 2. Pour chaque scanline (y), cherche toutes les intersections avec les segments
// 3. Exclut les sommets locaux maximums pour éviter le double comptage
// 4. Trie les abscisses d'intersection
// 5. Remplit entre chaque paire d'intersections
//
// Cette version gère correctement les polygones concaves et la plupart des cas complexes.
// -----------------------------------------------------------------------------
void fill_polygon320(Point2D* pts, int n, uint8_t color) {
    int i, j, y;
    int miny = SCREEN_HEIGHT-1, maxy = 0;

    // 1. Recherche du min et max Y du polygone
    for (i = 0; i < n; i++) {
        if (pts[i].y < miny) miny = pts[i].y; // plus haut
        if (pts[i].y > maxy) maxy = pts[i].y; // plus bas
    }

    // 2. Pour chaque scanline (ligne horizontale) entre miny et maxy
    for (y = miny; y <= maxy; y++) {
        int nodes = 0;
        int nodeX[16]; // Stocke les abscisses d'intersection pour cette ligne
        j = n - 1;

        // 3. Recherche des intersections entre la scanline et chaque segment du polygone
        // Solution robuste : n'exclure le sommet que s'il est un maximum local (plus haut que ses deux voisins)
        for (i = 0; i < n; i++) {
            int y0 = pts[i].y, y1 = pts[j].y;
            int x0 = pts[i].x, x1 = pts[j].x;
            int prev = (i == 0) ? n-1 : i-1;
            int next = (i+1 == n) ? 0 : i+1;
            int isLocalMax_i = (y0 > pts[prev].y) && (y0 > pts[next].y);
            int prev_j = (j == 0) ? n-1 : j-1;
            int next_j = (j+1 == n) ? 0 : j+1;
            int isLocalMax_j = (y1 > pts[prev_j].y) && (y1 > pts[next_j].y);
            if ((y0 < y && y1 >= y) || (y1 < y && y0 >= y)) {
                // Exclure le sommet si c'est un maximum local
                if ((y == y0 && isLocalMax_i) || (y == y1 && isLocalMax_j)) {
                    j = i; continue;
                }
                // Calcul de l'abscisse d'intersection par interpolation linéaire
                // x = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
                nodeX[nodes++] = x0 + (y - y0) * (x1 - x0) / (y1 - y0);
            }
            j = i;
        }

        // 4. Tri des abscisses d'intersection (ordre croissant)
        // (Nécessaire car les intersections ne sont pas forcément dans l'ordre)
        for (i = 0; i < nodes-1; i++) {
            for (j = i+1; j < nodes; j++) {
                if (nodeX[i] > nodeX[j]) {
                    int t = nodeX[i]; nodeX[i] = nodeX[j]; nodeX[j] = t;
                }
            }
        }

        // 5. Remplissage entre chaque paire d'intersections (x0,x1), (x2,x3), ...
        for (i = 0; i < nodes; i += 2) {
            if (i+1 < nodes) {
                int xstart = nodeX[i], xend = nodeX[i+1];
                // Clipping horizontal (évite de sortir de l'écran)
                if (xstart < 0) xstart = 0;
                if (xend >= SCREEN_WIDTH) xend = SCREEN_WIDTH-1;
                // Remplit tous les pixels entre xstart et xend sur la ligne y
                for (int x = xstart; x <= xend; x++) put_pixel320(x, y, color);
            }
        }
        // Si nodes est impair, il y a un problème de polygone non simple
    }
}

// -----------------------------------------------------------------------------
// Remplit un polygone puis trace son contour (utilise drawline320_asm si dispo)
// Prépare l'intégration future de fillpoly_asm
// -----------------------------------------------------------------------------

// Active l'utilisation de la version assembleur pour le tracé de ligne
#define USE_DRAWLINE320_ASM
extern void drawline320_asm(int x0, int y0, int x1, int y1, uint8_t color);
extern void fillpoly_asm(Point2D* pts, int n, uint8_t color);

void draw_filled_poly_with_border(Point2D* pts, int n, uint8_t fillcol, uint8_t bordercol) {
    // Remplissage : utiliser la version C ou assembler si dispo
    // fillpoly_asm(pts, n, fillcol); // Décommente quand prête
    fill_polygon320(pts, n, fillcol);
    if (n < 2) return;
    int x0 = pts[0].x, y0 = pts[0].y;
    for (int i = 1; i < n; ++i) {
        int x1 = pts[i].x, y1 = pts[i].y;
        // Utilise la version assembleur si dispo, sinon fallback C
        #ifdef USE_DRAWLINE320_ASM
        drawline320_asm(x0, y0, x1, y1, bordercol);
        #else
        draw_line320(x0, y0, x1, y1, bordercol);
        #endif
        x0 = x1; y0 = y1;
    }
    // Dernier segment (fermeture)
    #ifdef USE_DRAWLINE320_ASM
    drawline320_asm(x0, y0, pts[0].x, pts[0].y, bordercol);
    #else
    draw_line320(x0, y0, pts[0].x, pts[0].y, bordercol);
    #endif
}

