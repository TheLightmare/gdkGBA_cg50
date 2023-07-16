// GINT LIBS =====
#include <gint/display.h>
#include <gint/keyboard.h>

#include "gint_gba.h"
//===============

#include <stdio.h>
#include <stdlib.h>

#include "arm.h"
#include "arm_mem.h"

#include "io.h"

#include "video.h"


const int64_t max_rom_sz = 32 * 1024 * 1024;

static uint32_t to_pow2(uint32_t val) {
    val--;

    val |= (val >>  1);
    val |= (val >>  2);
    val |= (val >>  4);
    val |= (val >>  8);
    val |= (val >> 16);

    return val + 1;
}

int main(void) {
    //printf("gdkGBA - Gameboy Advance emulator made by gdkchan\n");
    //printf("This is FREE software released into the PUBLIC DOMAIN\n\n");
    dclear(C_WHITE);
    dtext(1, 20, C_BLACK, "gdkGBA - Gameboy Advance emulator made by gdkchan");
    dtext(1, 40, C_BLACK, "ported to fx-CG50 by Lightmare");
    dtext(1, 60, C_BLACK, "This is FREE software released into the PUBLIC DOMAIN");
    dupdate();

    getkey();

    gint_gba_init();
    arm_init();


    //TODO : make an actual ROM select menu
    
    /* if (argc < 2) {
        printf("Error: Invalid number of arguments!\n");
        printf("Please specify a ROM file.\n");

        return 0;
    } */


    FILE *image;

    image = fopen("gba_bios.bin", "rb");

    if (image == NULL) {
        //printf("Error: GBA BIOS not found!\n");
        //printf("Place it on this directory with the name \"gba_bios.bin\".\n");
        dclear(C_WHITE);
        dtext(1, 1, C_BLACK, "Error: GBA BIOS not found!");
        dtext(1, 20, C_BLACK, "Place it on this directory with the name \"gba_bios.bin\".");
        dupdate();

        getkey();

        return 0;
    }

    fread(bios, 16384, 1, image);

    fclose(image);

    // TODO : make so other games can be loaded, this is just for debug
    image = fopen("test.gba", "rb");

    if (image == NULL) {
        dclear(C_WHITE);
        dtext(1, 1, C_BLACK, "Error: ROM file couldn't be opened.\n");
        dtext(1, 20, C_BLACK, "Make sure that the file exists and the name is correct.\n");
        dupdate();

        getkey();

        return 0;
    }

    //DEBUG
    dclear(C_WHITE);
    dtext(1, 1, C_BLACK, "loaded ROM image");
    dupdate();
    getkey();
    //====

    fseek(image, 0, SEEK_END);

    cart_rom_size = ftell(image);

    //DEBUG
    dclear(C_WHITE);
    dprint(1, 1, C_BLACK, "cart_rom_size : %ld bytes", cart_rom_size);
    dupdate();
    getkey();
    //====

    cart_rom_mask = to_pow2(cart_rom_size) - 1;

    if (cart_rom_size > max_rom_sz) cart_rom_size = max_rom_sz;

    fseek(image, 0, SEEK_SET);

    //DEBUG
    dclear(C_WHITE);
    dtext(1, 1, C_BLACK, "reading ROM image...");
    dupdate();
    getkey();
    //====

    // THIS MAKES EVERYTHING CRASH, try to find better alternative.
    fread(rom, cart_rom_size, 1, image);

    fclose(image);

    arm_reset();


    bool run = true;

    while (run) {
        run_frame();

        //SDL_Event event;
        int opt = GETKEY_DEFAULT & ~GETKEY_REP_ARROWS;
        int timeout = 1;
        key_event_t event = getkey_opt(opt, &timeout);
        int key = event.key;
        if(key == KEY_UP)    key_input.w &= ~BTN_U;
        if(key == KEY_DOWN)  key_input.w &= ~BTN_D;        
        if(key == KEY_LEFT)  key_input.w &= ~BTN_L;
        if(key == KEY_RIGHT) key_input.w &= ~BTN_R;

        if(key == KEY_MENU) run = false;

        

        // TODO : replace the event system with the Gint one
        /* while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_KEYDOWN:
                    switch (event.key.keysym.sym) {
                        case SDLK_UP:     key_input.w &= ~BTN_U;   break;
                        case SDLK_DOWN:   key_input.w &= ~BTN_D;   break;
                        case SDLK_LEFT:   key_input.w &= ~BTN_L;   break;
                        case SDLK_RIGHT:  key_input.w &= ~BTN_R;   break;
                        case SDLK_a:      key_input.w &= ~BTN_A;   break;
                        case SDLK_s:      key_input.w &= ~BTN_B;   break;
                        case SDLK_q:      key_input.w &= ~BTN_LT;  break;
                        case SDLK_w:      key_input.w &= ~BTN_RT;  break;
                        case SDLK_TAB:    key_input.w &= ~BTN_SEL; break;
                        case SDLK_RETURN: key_input.w &= ~BTN_STA; break;
                        default:                                   break;
                    }
                break;

                case SDL_KEYUP:
                    switch (event.key.keysym.sym) {
                        case SDLK_UP:     key_input.w |= BTN_U;   break;
                        case SDLK_DOWN:   key_input.w |= BTN_D;   break;
                        case SDLK_LEFT:   key_input.w |= BTN_L;   break;
                        case SDLK_RIGHT:  key_input.w |= BTN_R;   break;
                        case SDLK_a:      key_input.w |= BTN_A;   break;
                        case SDLK_s:      key_input.w |= BTN_B;   break;
                        case SDLK_q:      key_input.w |= BTN_LT;  break;
                        case SDLK_w:      key_input.w |= BTN_RT;  break;
                        case SDLK_TAB:    key_input.w |= BTN_SEL; break;
                        case SDLK_RETURN: key_input.w |= BTN_STA; break;
                        default:                                  break;
                    }
                break;

                case SDL_QUIT: run = false; break;
            }
        } */
    }

    gint_gba_uninit();
    arm_uninit();

    return 1;
}