// file = 0; split type = patterns; threshold = 100000; total count = 0.
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include "rmapats.h"

void  hsG_0__0 (struct dummyq_struct * I1199, EBLK  * I1193, U  I670);
void  hsG_0__0 (struct dummyq_struct * I1199, EBLK  * I1193, U  I670)
{
    U  I1450;
    U  I1451;
    U  I1452;
    struct futq * I1453;
    struct dummyq_struct * pQ = I1199;
    I1450 = ((U )vcs_clocks) + I670;
    I1452 = I1450 & ((1 << fHashTableSize) - 1);
    I1193->I708 = (EBLK  *)(-1);
    I1193->I712 = I1450;
    if (I1450 < (U )vcs_clocks) {
        I1451 = ((U  *)&vcs_clocks)[1];
        sched_millenium(pQ, I1193, I1451 + 1, I1450);
    }
    else if ((peblkFutQ1Head != ((void *)0)) && (I670 == 1)) {
        I1193->I714 = (struct eblk *)peblkFutQ1Tail;
        peblkFutQ1Tail->I708 = I1193;
        peblkFutQ1Tail = I1193;
    }
    else if ((I1453 = pQ->I1103[I1452].I726)) {
        I1193->I714 = (struct eblk *)I1453->I725;
        I1453->I725->I708 = (RP )I1193;
        I1453->I725 = (RmaEblk  *)I1193;
    }
    else {
        sched_hsopt(pQ, I1193, I1450);
    }
}
#ifdef __cplusplus
extern "C" {
#endif
void SinitHsimPats(void);
#ifdef __cplusplus
}
#endif
