#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ff.h"  // FatFS library header for SD card

#define MAX_SONG 10
#define MAX_TITLE_LENGTH 25

// global variable from FatFs
FRESULT res; 
DIR dir; // structure for the directory
FILINFO fno; // File information structure

// structure of song list from WAV file
typedef struct {
  char song_title[MAX_SONG][MAX_TITLE_LENGTH];  
  int count;
} songList;

void init_list(songList *list);
void add_song(songList *list, const char *title, size_t title_length);
void make_list_from_sd(songList *list, const char *directory);
void display_song_list(const songList *list);

// 1. initialize song list
void init_list(songList *list){
  list->count = 0;
}

// 2. add songs to the song_list
void add_song(songList *list, const char *title, size_t title_length){ 
  if (list->count < MAX_SONG) {
    strncpy(list->song_title[list->count], title, title_length);
    list->song_title[list->count][title_length] = '\0';  // 마지막 인덱스에 널 문자 추가
    list->count++; 
  } else {
    printf("cannot add more songs.\n");
  }
}

void display_song_list(const songList *list) {
  for (int i = 0; i < list->count; i++) {
    printf("%d: %s\n", i + 1, list->song_title[i]);
  }
}

void make_list_from_sd(songList *list, const char *directory) {
    init_list(list);

    res = f_opendir(&dir, directory); 
    if (res == FR_OK) {
        while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0 && list->count < MAX_SONG) {
            const char *filename = fno.fname;  
            const char *extension = strrchr(filename, '.');  
            if (extension != NULL && strcmp(extension, ".wav") == 0) {  
                size_t title_length = extension - filename;  
                add_song(list, filename, title_length);  
            }
        f_closedir(&dir);
        } 
    }
    else 
    {
        printf("cannot open directory\n");
    }
}

