#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;

extern void _IhDA_reg(void);
extern void _ampasyn_reg(void);
extern void _cabalan_reg(void);
extern void _cachan_reg(void);
extern void _capump_reg(void);
extern void _dop_reg(void);
extern void _hh3_reg(void);
extern void _kca_reg(void);
extern void _leak_reg(void);
extern void _nabalan_reg(void);
extern void _nmdasyn_reg(void);
extern void _pump_reg(void);

void modl_reg(){
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");

    fprintf(stderr," mechanisms/IhDA.mod");
    fprintf(stderr," mechanisms/ampasyn.mod");
    fprintf(stderr," mechanisms/cabalan.mod");
    fprintf(stderr," mechanisms/cachan.mod");
    fprintf(stderr," mechanisms/capump.mod");
    fprintf(stderr," mechanisms/dop.mod");
    fprintf(stderr," mechanisms/hh3.mod");
    fprintf(stderr," mechanisms/kca.mod");
    fprintf(stderr," mechanisms/leak.mod");
    fprintf(stderr," mechanisms/nabalan.mod");
    fprintf(stderr," mechanisms/nmdasyn.mod");
    fprintf(stderr," mechanisms/pump.mod");
    fprintf(stderr, "\n");
  }
  _IhDA_reg();
  _ampasyn_reg();
  _cabalan_reg();
  _cachan_reg();
  _capump_reg();
  _dop_reg();
  _hh3_reg();
  _kca_reg();
  _leak_reg();
  _nabalan_reg();
  _nmdasyn_reg();
  _pump_reg();
}
