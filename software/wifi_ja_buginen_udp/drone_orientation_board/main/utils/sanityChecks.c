#include "sanityChecks.h"

void print(char text[] ){
    char buff[50];
    snprintf(buff,50,"%s\n",text);
    printf("%s",buff);
}