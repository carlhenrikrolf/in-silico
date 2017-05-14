/* Created by Language version: 6.2.0 */
/* NOT VECTORIZED */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define _threadargscomma_ /**/
#define _threadargs_ /**/
 
#define _threadargsprotocomma_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define gcatbar _p[0]
#define gcanbar _p[1]
#define gcalbar _p[2]
#define kmn _p[3]
#define kml _p[4]
#define ica _p[5]
#define ical _p[6]
#define ican _p[7]
#define icat _p[8]
#define d_t _p[9]
#define dl _p[10]
#define dn _p[11]
#define ft _p[12]
#define cai _p[13]
#define Dd_t _p[14]
#define Ddl _p[15]
#define Ddn _p[16]
#define Dft _p[17]
#define _g _p[18]
#define _ion_cai	*_ppvar[0]._pval
#define _ion_ica	*_ppvar[1]._pval
#define _ion_dicadv	*_ppvar[2]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_boltz(void);
 static void _hoc_gaussian(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _p = _prop->param; _ppvar = _prop->dparam;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_cachan", _hoc_setdata,
 "boltz_cachan", _hoc_boltz,
 "gaussian_cachan", _hoc_gaussian,
 0, 0
};
#define boltz boltz_cachan
#define gaussian gaussian_cachan
 extern double boltz( double , double , double );
 extern double gaussian( double , double , double , double , double );
 /* declare global and static user variables */
#define cao cao_cachan
 double cao = 2;
#define dtinf dtinf_cachan
 double dtinf = 0;
#define dninf dninf_cachan
 double dninf = 0;
#define dlinf dlinf_cachan
 double dlinf = 0;
#define eca eca_cachan
 double eca = 120;
#define fninf fninf_cachan
 double fninf = 0;
#define flinf flinf_cachan
 double flinf = 0;
#define ftinf ftinf_cachan
 double ftinf = 0;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "eca_cachan", "mV",
 "cao_cachan", "mM",
 "gcatbar_cachan", "S/cm2",
 "gcanbar_cachan", "S/cm2",
 "gcalbar_cachan", "S/cm2",
 "kmn_cachan", "mM",
 "kml_cachan", "mM",
 "ica_cachan", "mA/cm2",
 "ical_cachan", "mA/cm2",
 "ican_cachan", "mA/cm2",
 "icat_cachan", "mA/cm2",
 0,0
};
 static double delta_t = 1;
 static double dn0 = 0;
 static double dl0 = 0;
 static double d_t0 = 0;
 static double ft0 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "eca_cachan", &eca_cachan,
 "cao_cachan", &cao_cachan,
 "dlinf_cachan", &dlinf_cachan,
 "dninf_cachan", &dninf_cachan,
 "dtinf_cachan", &dtinf_cachan,
 "ftinf_cachan", &ftinf_cachan,
 "fninf_cachan", &fninf_cachan,
 "flinf_cachan", &flinf_cachan,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[3]._i
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"cachan",
 "gcatbar_cachan",
 "gcanbar_cachan",
 "gcalbar_cachan",
 "kmn_cachan",
 "kml_cachan",
 0,
 "ica_cachan",
 "ical_cachan",
 "ican_cachan",
 "icat_cachan",
 0,
 "d_t_cachan",
 "dl_cachan",
 "dn_cachan",
 "ft_cachan",
 0,
 0};
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 19, _prop);
 	/*initialize range parameters*/
 	gcatbar = 0.001044;
 	gcanbar = 0.000171;
 	gcalbar = 0.000216;
 	kmn = 0.0001;
 	kml = 0.00045;
 	_prop->param = _p;
 	_prop->param_size = 19;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[0]._pval = &prop_ion->param[1]; /* cai */
 	_ppvar[1]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[2]._pval = &prop_ion->param[4]; /* _ion_dicadv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*f)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _cachan_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("ca", -10000.);
 	_ca_sym = hoc_lookup("ca_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 0);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
  hoc_register_prop_size(_mechtype, 19, 4);
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 cachan /home/hbpschool2016/Documents/cell_optimization/x86_64/cachan.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double F = 96485.3;
 static double R = 8313.42;
static int _reset;
static char *modelname = "calcium channels (L, N, and T types) ";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[4], _dlist1[4];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   double _ldlinf , _ldninf , _ldtinf , _lftinf , _ldltau , _ldntau , _ldttau , _lfttau ;
 _ldlinf = boltz ( _threadargscomma_ v , - 50.0 , 3.0 ) ;
   _ldninf = boltz ( _threadargscomma_ v , - 45.0 , 7.0 ) ;
   _ldtinf = boltz ( _threadargscomma_ v , - 63.5 , 1.5 ) ;
   _lftinf = boltz ( _threadargscomma_ v , - 76.2 , - 3.0 ) ;
   _ldltau = gaussian ( _threadargscomma_ v , 18.0 , 20.0 , 45.0 , 1.50 ) ;
   _ldntau = gaussian ( _threadargscomma_ v , 18.0 , 25.0 , 70.0 , 0.30 ) ;
   _ldttau = gaussian ( _threadargscomma_ v , 65.0 , 6.32455 , 66.0 , 3.5 ) ;
   _lfttau = gaussian ( _threadargscomma_ v , 50.0 , 10.0 , 72.0 , 10.0 ) ;
   Ddl = ( _ldlinf - dl ) / _ldltau ;
   Ddn = ( _ldninf - dn ) / _ldntau ;
   Dd_t = ( _ldtinf - d_t ) / _ldttau ;
   Dft = ( _lftinf - ft ) / _lfttau ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 double _ldlinf , _ldninf , _ldtinf , _lftinf , _ldltau , _ldntau , _ldttau , _lfttau ;
 _ldlinf = boltz ( _threadargscomma_ v , - 50.0 , 3.0 ) ;
 _ldninf = boltz ( _threadargscomma_ v , - 45.0 , 7.0 ) ;
 _ldtinf = boltz ( _threadargscomma_ v , - 63.5 , 1.5 ) ;
 _lftinf = boltz ( _threadargscomma_ v , - 76.2 , - 3.0 ) ;
 _ldltau = gaussian ( _threadargscomma_ v , 18.0 , 20.0 , 45.0 , 1.50 ) ;
 _ldntau = gaussian ( _threadargscomma_ v , 18.0 , 25.0 , 70.0 , 0.30 ) ;
 _ldttau = gaussian ( _threadargscomma_ v , 65.0 , 6.32455 , 66.0 , 3.5 ) ;
 _lfttau = gaussian ( _threadargscomma_ v , 50.0 , 10.0 , 72.0 , 10.0 ) ;
 Ddl = Ddl  / (1. - dt*( ( ( ( - 1.0 ) ) ) / _ldltau )) ;
 Ddn = Ddn  / (1. - dt*( ( ( ( - 1.0 ) ) ) / _ldntau )) ;
 Dd_t = Dd_t  / (1. - dt*( ( ( ( - 1.0 ) ) ) / _ldttau )) ;
 Dft = Dft  / (1. - dt*( ( ( ( - 1.0 ) ) ) / _lfttau )) ;
 return 0;
}
 /*END CVODE*/
 static int states () {_reset=0;
 {
   double _ldlinf , _ldninf , _ldtinf , _lftinf , _ldltau , _ldntau , _ldttau , _lfttau ;
 _ldlinf = boltz ( _threadargscomma_ v , - 50.0 , 3.0 ) ;
   _ldninf = boltz ( _threadargscomma_ v , - 45.0 , 7.0 ) ;
   _ldtinf = boltz ( _threadargscomma_ v , - 63.5 , 1.5 ) ;
   _lftinf = boltz ( _threadargscomma_ v , - 76.2 , - 3.0 ) ;
   _ldltau = gaussian ( _threadargscomma_ v , 18.0 , 20.0 , 45.0 , 1.50 ) ;
   _ldntau = gaussian ( _threadargscomma_ v , 18.0 , 25.0 , 70.0 , 0.30 ) ;
   _ldttau = gaussian ( _threadargscomma_ v , 65.0 , 6.32455 , 66.0 , 3.5 ) ;
   _lfttau = gaussian ( _threadargscomma_ v , 50.0 , 10.0 , 72.0 , 10.0 ) ;
    dl = dl + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / _ldltau)))*(- ( ( ( _ldlinf ) ) / _ldltau ) / ( ( ( ( - 1.0) ) ) / _ldltau ) - dl) ;
    dn = dn + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / _ldntau)))*(- ( ( ( _ldninf ) ) / _ldntau ) / ( ( ( ( - 1.0) ) ) / _ldntau ) - dn) ;
    d_t = d_t + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / _ldttau)))*(- ( ( ( _ldtinf ) ) / _ldttau ) / ( ( ( ( - 1.0) ) ) / _ldttau ) - d_t) ;
    ft = ft + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / _lfttau)))*(- ( ( ( _lftinf ) ) / _lfttau ) / ( ( ( ( - 1.0) ) ) / _lfttau ) - ft) ;
   }
  return 0;
}
 
double gaussian (  double _lv , double _la , double _lb , double _lc , double _ld ) {
   double _lgaussian;
 double _larg ;
 _larg = _la * exp ( - ( _lc + _lv ) * ( _lv + _lc ) / ( _lb * _lb ) ) + _ld ;
   _lgaussian = _larg ;
   
return _lgaussian;
 }
 
static void _hoc_gaussian(void) {
  double _r;
   _r =  gaussian (  *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) , *getarg(5) );
 hoc_retpushx(_r);
}
 
double boltz (  double _lx , double _ly , double _lz ) {
   double _lboltz;
 double _larg ;
 _larg = - ( _lx - _ly ) / _lz ;
   if ( _larg > 50.0 ) {
     _lboltz = 0.0 ;
     }
   else {
     if ( _larg < - 50.0 ) {
       _lboltz = 1.0 ;
       }
     else {
       _lboltz = 1.0 / ( 1.0 + exp ( _larg ) ) ;
       }
     }
   
return _lboltz;
 }
 
static void _hoc_boltz(void) {
  double _r;
   _r =  boltz (  *getarg(1) , *getarg(2) , *getarg(3) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 4;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
     _ode_spec1 ();
  }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 4; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
 _ode_matsol1 ();
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 0, 1);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 1, 3);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 2, 4);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  dn = dn0;
  dl = dl0;
  d_t = d_t0;
  ft = ft0;
 {
   dl = boltz ( _threadargscomma_ v , - 50.0 , 3.0 ) ;
   dn = boltz ( _threadargscomma_ v , - 45.0 , 7.0 ) ;
   d_t = boltz ( _threadargscomma_ v , - 63.5 , 1.5 ) ;
   ft = boltz ( _threadargscomma_ v , - 76.2 , - 3.0 ) ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
  cai = _ion_cai;
 initmodel();
 }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   fninf = kmn / ( kmn + cai ) ;
   flinf = kml / ( kml + cai ) ;
   ical = gcalbar * dl * flinf * ( v - eca ) ;
   ican = gcanbar * dn * fninf * ( v - eca ) ;
   icat = gcatbar * d_t * ft * ( v - eca ) ;
   ica = ( ical + ican + icat ) ;
   }
 _current += ica;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
  cai = _ion_cai;
 _g = _nrn_current(_v + .001);
 	{ double _dica;
  _dica = ica;
 _rhs = _nrn_current(_v);
  _ion_dicadv += (_dica - ica)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ica += ica ;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type){
 double _break, _save;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _break = t + .5*dt; _save = t;
 v=_v;
{
  cai = _ion_cai;
 { {
 for (; t < _break; t += dt) {
 error =  states();
 if(error){fprintf(stderr,"at line 51 in file cachan.mod:\n        SOLVE states METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
 
}}
 t = _save;
 } }}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(dl) - _p;  _dlist1[0] = &(Ddl) - _p;
 _slist1[1] = &(dn) - _p;  _dlist1[1] = &(Ddn) - _p;
 _slist1[2] = &(d_t) - _p;  _dlist1[2] = &(Dd_t) - _p;
 _slist1[3] = &(ft) - _p;  _dlist1[3] = &(Dft) - _p;
_first = 0;
}
