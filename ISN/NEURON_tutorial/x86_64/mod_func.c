#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;

extern void _TC_HH_reg(void);
extern void _TC_ITGHK_reg(void);
extern void _TC_Ih_reg(void);
extern void _TC_iA_reg(void);

void modl_reg(){
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");

    fprintf(stderr," TC_HH.mod");
    fprintf(stderr," TC_ITGHK.mod");
    fprintf(stderr," TC_Ih.mod");
    fprintf(stderr," TC_iA.mod");
    fprintf(stderr, "\n");
  }
  _TC_HH_reg();
  _TC_ITGHK_reg();
  _TC_Ih_reg();
  _TC_iA_reg();
}
