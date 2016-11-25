#ifndef VIV2D_EXA_H
#define VIV2D_EXA_H

#include "omap_driver.h"
#include "omap_exa.h"

typedef struct {
	OMAPEXARec base;
	ExaDriverPtr exa;
	/* add any other driver private data here.. */
	Viv2DPtr v2d;
} Viv2DEXARec, *Viv2DEXAPtr;


static Viv2DRec*
Viv2DPrivFromPixmap(PixmapPtr pPixmap)
{
	ScrnInfoPtr pScrn = pix2scrn(pPixmap);
	OMAPPtr pOMAP = OMAPPTR(pScrn);
	Viv2DEXAPtr exa = (Viv2DEXAPtr)(pOMAP->pOMAPEXA);
	Viv2DRec *v2d = exa->v2d;
	return v2d;
}

static Viv2DRec*
Viv2DPrivFromScreen(ScreenPtr pScreen)
{
	Viv2DRec *v2d;
	Viv2DEXAPtr exa;
	OMAPPtr pOMAP = OMAPPTR_FROM_SCREEN(pScreen);
	exa = (Viv2DEXAPtr)(pOMAP->pOMAPEXA);
	v2d = exa->v2d;
	return v2d;
}

#endif