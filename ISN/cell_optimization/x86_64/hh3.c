/* Created by Language version: 6.2.0 */
/* VECTORIZED */
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
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
 
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define gnabar _p[0]
#define gkhhbar _p[1]
#define gkabar _p[2]
#define miv _p[3]
#define hiv _p[4]
#define htv1 _p[5]
#define htv2 _p[6]
#define ina _p[7]
#define ik _p[8]
#define ika _p[9]
#define ikhh _p[10]
#define ena _p[11]
#define m _p[12]
#define h _p[13]
#define n _p[14]
#define p _p[15]
#define q _p[16]
#define nai _p[17]
#define Dm _p[18]
#define Dh _p[19]
#define Dn _p[20]
#define Dp _p[21]
#define Dq _p[22]
#define qinf _p[23]
#define pinf _p[24]
#define v _p[25]
#define _g _p[26]
#define _ion_nai	*_ppvar[0]._pval
#define _ion_ina	*_ppvar[1]._pval
#define _ion_dinadv	*_ppvar[2]._pval
#define _ion_ik	*_ppvar[3]._pval
#define _ion_dikdv	*_ppvar[4]._pval
 
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
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_boltz(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_hh3", _hoc_setdata,
 "boltz_hh3", _hoc_boltz,
 0, 0
};
#define boltz boltz_hh3
 extern double boltz( _threadargsprotocomma_ double , double , double );
 /* declare global and static user variables */
#define ek ek_hh3
 double ek = -100;
#define hinf hinf_hh3
 double hinf = 0;
#define minf minf_hh3
 double minf = 0;
#define nao nao_hh3
 double nao = 145;
#define ninf ninf_hh3
 double ninf = 0;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "ek_hh3", "mV",
 "nao_hh3", "mM",
 "gnabar_hh3", "S/cm2",
 "gkhhbar_hh3", "S/cm2",
 "gkabar_hh3", "S/cm2",
 "miv_hh3", "mV",
 "hiv_hh3", "mV",
 "htv1_hh3", "mV",
 "htv2_hh3", "mV",
 "ina_hh3", "mA/cm2",
 "ik_hh3", "mA/cm2",
 "ika_hh3", "mA/cm2",
 "ikhh_hh3", "mA/cm2",
 "ena_hh3", "mV",
 0,0
};
 static double delta_t = 1;
 static double h0 = 0;
 static double m0 = 0;
 static double n0 = 0;
 static double p0 = 0;
 static double q0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "ek_hh3", &ek_hh3,
 "nao_hh3", &nao_hh3,
 "minf_hh3", &minf_hh3,
 "hinf_hh3", &hinf_hh3,
 "ninf_hh3", &ninf_hh3,
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
 
#define _cvode_ieq _ppvar[5]._i
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"hh3",
 "gnabar_hh3",
 "gkhhbar_hh3",
 "gkabar_hh3",
 "miv_hh3",
 "hiv_hh3",
 "htv1_hh3",
 "htv2_hh3",
 0,
 "ina_hh3",
 "ik_hh3",
 "ika_hh3",
 "ikhh_hh3",
 "ena_hh3",
 0,
 "m_hh3",
 "h_hh3",
 "n_hh3",
 "p_hh3",
 "q_hh3",
 0,
 0};
 static Symbol* _na_sym;
 static Symbol* _k_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 27, _prop);
 	/*initialize range parameters*/
 	gnabar = 0.0055;
 	gkhhbar = 0.001;
 	gkabar = 0.0001;
 	miv = 44.6;
 	hiv = 66.8;
 	htv1 = 39;
 	htv2 = 59;
 	_prop->param = _p;
 	_prop->param_size = 27;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 6, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_na_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[0]._pval = &prop_ion->param[1]; /* nai */
 	_ppvar[1]._pval = &prop_ion->param[3]; /* ina */
 	_ppvar[2]._pval = &prop_ion->param[4]; /* _ion_dinadv */
 prop_ion = need_memb(_k_sym);
 	_ppvar[3]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[4]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 "m_hh3", 0.0001,
 "h_hh3", 0.0001,
 "n_hh3", 0.0001,
 "p_hh3", 0.0001,
 "q_hh3", 0.0001,
 0,0
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*f)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _hh3_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("na", -10000.);
 	ion_reg("k", -10000.);
 	_na_sym = hoc_lookup("na_ion");
 	_k_sym = hoc_lookup("k_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
  hoc_register_prop_size(_mechtype, 27, 6);
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 hh3 /home/hbpschool2016/Documents/cell_optimization/x86_64/hh3.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double F = 96485.3;
 static double R = 8313.42;
static int _reset;
static char *modelname = "squid sodium, potassium delayed rectifier, and potassium A channels";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[5], _dlist1[5];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   double _lminf , _lhinf , _lninf , _lpinf , _lqinf , _lmtau , _lhtau , _lntau , _lptau , _lqtau ;
 _lminf = boltz ( _threadargscomma_ v , - miv , 6.0 ) ;
   _lhinf = boltz ( _threadargscomma_ v , - hiv , - 7.8 ) ;
   _lninf = boltz ( _threadargscomma_ v , - 35.0 , 12.0 ) ;
   _lpinf = boltz ( _threadargscomma_ v , - 42.0 , 4.0 ) ;
   _lqinf = boltz ( _threadargscomma_ v , - 63.0 , - 4.0 ) ;
   _lmtau = boltz ( _threadargscomma_ v , - 45.0 , - 1.5 ) - boltz ( _threadargscomma_ v , - 65.0 , - 0.5 ) + 0.04 ;
   _lhtau = 56.0 * boltz ( _threadargscomma_ v , - htv1 , - 4.5 ) - 56.0 * boltz ( _threadargscomma_ v , - htv2 , - 2.0 ) + 1.0 ;
   _lntau = 10.0 ;
   _lptau = 5.5 * exp ( - ( v + 42.0 ) * ( v + 42.0 ) / 100.0 ) + 4.0 ;
   _lqtau = 50.0 ;
   Dm = ( _lminf - m ) / _lmtau ;
   Dh = ( _lhinf - h ) / _lhtau ;
   Dn = ( _lninf - n ) / _lntau ;
   Dp = ( _lpinf - p ) / _lptau ;
   Dq = ( _lqinf - q ) / _lqtau ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 double _lminf , _lhinf , _lninf , _lpinf , _lqinf , _lmtau , _lhtau , _lntau , _lptau , _lqtau ;
 _lminf = boltz ( _threadargscomma_ v , - miv , 6.0 ) ;
 _lhinf = boltz ( _threadargscomma_ v , - hiv , - 7.8 ) ;
 _lninf = boltz ( _threadargscomma_ v , - 35.0 , 12.0 ) ;
 _lpinf = boltz ( _threadargscomma_ v , - 42.0 , 4.0 ) ;
 _lqinf = boltz ( _threadargscomma_ v , - 63.0 , - 4.0 ) ;
 _lmtau = boltz ( _threadargscomma_ v , - 45.0 , - 1.5 ) - boltz ( _threadargscomma_ v , - 65.0 , - 0.5 ) + 0.04 ;
 _lhtau = 56.0 * boltz ( _threadargscomma_ v , - htv1 , - 4.5 ) - 56.0 * boltz ( _threadargscomma_ v , - htv2 , - 2.0 ) + 1.0 ;
 _lntau = 10.0 ;
 _lptau = 5.5 * exp ( - ( v + 42.0 ) * ( v + 42.0 ) / 100.0 ) + 4.0 ;
 _lqtau = 50.0 ;
 Dm = Dm  / (1. - dt*( ( ( ( - 1.0 ) ) ) / _lmtau )) ;
 Dh = Dh  / (1. - dt*( ( ( ( - 1.0 ) ) ) / _lhtau )) ;
 Dn = Dn  / (1. - dt*( ( ( ( - 1.0 ) ) ) / _lntau )) ;
 Dp = Dp  / (1. - dt*( ( ( ( - 1.0 ) ) ) / _lptau )) ;
 Dq = Dq  / (1. - dt*( ( ( ( - 1.0 ) ) ) / _lqtau )) ;
 return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   double _lminf , _lhinf , _lninf , _lpinf , _lqinf , _lmtau , _lhtau , _lntau , _lptau , _lqtau ;
 _lminf = boltz ( _threadargscomma_ v , - miv , 6.0 ) ;
   _lhinf = boltz ( _threadargscomma_ v , - hiv , - 7.8 ) ;
   _lninf = boltz ( _threadargscomma_ v , - 35.0 , 12.0 ) ;
   _lpinf = boltz ( _threadargscomma_ v , - 42.0 , 4.0 ) ;
   _lqinf = boltz ( _threadargscomma_ v , - 63.0 , - 4.0 ) ;
   _lmtau = boltz ( _threadargscomma_ v , - 45.0 , - 1.5 ) - boltz ( _threadargscomma_ v , - 65.0 , - 0.5 ) + 0.04 ;
   _lhtau = 56.0 * boltz ( _threadargscomma_ v , - htv1 , - 4.5 ) - 56.0 * boltz ( _threadargscomma_ v , - htv2 , - 2.0 ) + 1.0 ;
   _lntau = 10.0 ;
   _lptau = 5.5 * exp ( - ( v + 42.0 ) * ( v + 42.0 ) / 100.0 ) + 4.0 ;
   _lqtau = 50.0 ;
    m = m + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / _lmtau)))*(- ( ( ( _lminf ) ) / _lmtau ) / ( ( ( ( - 1.0) ) ) / _lmtau ) - m) ;
    h = h + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / _lhtau)))*(- ( ( ( _lhinf ) ) / _lhtau ) / ( ( ( ( - 1.0) ) ) / _lhtau ) - h) ;
    n = n + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / _lntau)))*(- ( ( ( _lninf ) ) / _lntau ) / ( ( ( ( - 1.0) ) ) / _lntau ) - n) ;
    p = p + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / _lptau)))*(- ( ( ( _lpinf ) ) / _lptau ) / ( ( ( ( - 1.0) ) ) / _lptau ) - p) ;
    q = q + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / _lqtau)))*(- ( ( ( _lqinf ) ) / _lqtau ) / ( ( ( ( - 1.0) ) ) / _lqtau ) - q) ;
   }
  return 0;
}
 
double boltz ( _threadargsprotocomma_ double _lx , double _ly , double _lz ) {
   double _lboltz;
 _lboltz = 1.0 / ( 1.0 + exp ( - ( _lx - _ly ) / _lz ) ) ;
   
return _lboltz;
 }
 
static void _hoc_boltz(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  boltz ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) , *getarg(3) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 5;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  nai = _ion_nai;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
   }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 5; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  nai = _ion_nai;
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_na_sym, _ppvar, 0, 1);
   nrn_update_ion_pointer(_na_sym, _ppvar, 1, 3);
   nrn_update_ion_pointer(_na_sym, _ppvar, 2, 4);
   nrn_update_ion_pointer(_k_sym, _ppvar, 3, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 4, 4);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  h = h0;
  m = m0;
  n = n0;
  p = p0;
  q = q0;
 {
   m = boltz ( _threadargscomma_ v , - miv , 6.0 ) ;
   h = boltz ( _threadargscomma_ v , - hiv , - 7.8 ) ;
   n = boltz ( _threadargscomma_ v , - 35.0 , 12.0 ) ;
   p = boltz ( _threadargscomma_ v , - 42.0 , 4.0 ) ;
   q = boltz ( _threadargscomma_ v , - 63.0 , - 4.0 ) ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
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
  nai = _ion_nai;
 initmodel(_p, _ppvar, _thread, _nt);
  }}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   ena = R * ( celsius + 273.15 ) / F * log ( nao / nai ) ;
   ina = gnabar * m * m * m * h * ( v - ena ) ;
   ikhh = gkhhbar * n * n * n * ( v - ek ) ;
   ika = gkabar * p * p * p * q * ( v - ek ) ;
   ik = ika + ikhh ;
   }
 _current += ina;
 _current += ik;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
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
  nai = _ion_nai;
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ double _dik;
 double _dina;
  _dina = ina;
  _dik = ik;
 _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
  _ion_dinadv += (_dina - ina)/.001 ;
  _ion_dikdv += (_dik - ik)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ina += ina ;
  _ion_ik += ik ;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
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

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
 double _break, _save;
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
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
  nai = _ion_nai;
 { {
 for (; t < _break; t += dt) {
   states(_p, _ppvar, _thread, _nt);
  
}}
 t = _save;
 }  }}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(m) - _p;  _dlist1[0] = &(Dm) - _p;
 _slist1[1] = &(h) - _p;  _dlist1[1] = &(Dh) - _p;
 _slist1[2] = &(n) - _p;  _dlist1[2] = &(Dn) - _p;
 _slist1[3] = &(p) - _p;  _dlist1[3] = &(Dp) - _p;
 _slist1[4] = &(q) - _p;  _dlist1[4] = &(Dq) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
