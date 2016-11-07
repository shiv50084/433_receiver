#include <stdio.h>

int main()
{
    unsigned long time1, time2;
    unsigned int level1, level2;
    ssize_t read;
    unsigned int l = 0, b = 0;
    char *line1 = NULL, *line2 = NULL;
    size_t len = 0;
    FILE *f = fopen("signal.dat", "r");
    while (getline(&line1, &len, f) != -1) {
        if (getline(&line2, &len, f) == -1) {
            printf("\nNo even lines left!\n");
            break;
        }
        if (sscanf(line1, "%lu %u", &time1, &level1) != EOF &&
            sscanf(line2, "%lu %u", &time2, &level2) != EOF) {
            if (level1 == level2) {
                printf("\nWrong level sequence at line %u!\n", l);
                break;
            }
            
            if (time2 > 2000) {
                b = 0;
                printf("\n");
                l += 2;
                continue;
            }
            //printf("Got %lu usecs for level %u\n", time1, level1);
            if (time1 > time2)
                printf("1");
            else
                printf("0");
            b++;
            if (b == 24) {
                b = 0;
                printf("\n");
                printf("24 bits at %u\n", l + 2);
                for (int i = 0; i < 2; i++) {
                    getline(&line1, &len, f);
                    l++;
                }
            }
        }
        l += 2;
    }
    fclose(f);
}