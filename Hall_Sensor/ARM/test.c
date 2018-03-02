//
// Created by dhan64 on 3/1/2018.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

float ss = 123.456789;
char payload[23];

int main(int argc, char* argv[]){
    printf("%d\n", strlen(payload));
    sprintf(payload, "spindle speed: %.2frpm\n",ss);
    printf("%d\n", strlen(payload));
    return 0;
}