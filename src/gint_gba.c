#include <gint/image.h>
#include <gint/display.h>
#include "gint_gba.h"

image_t* texture;

void gint_gba_init(){

    texture = image_alloc(240, 160, IMAGE_RGB565);
}

void gint_gba_uninit(){
    image_free(texture);
}