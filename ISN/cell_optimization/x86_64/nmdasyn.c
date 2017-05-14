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
#define Pbar _p[0]
#define pr _p[1]
#define km _p[2]
#define mg _p[3]
#define ica _p[4]
#define ina _p[5]
#define ik _p[6]
#define inmda _p[7]
#define p _p[8]
#define Dp _p[9]
#define nai _p[10]
#define _g _p[11]
#define _ion_ica	*_ppvar[0]._pval
#define _ion_dicadv	*_ppvar[1]._pval
#define _ion_nai	*_ppvar[2]._pval
#define _ion_ina	*_ppvar[3]._pval
#define _ion_dinadv	*_ppvar[4]._pval
#define _ion_ik	*_ppvar[5]._pval
#define _ion_dikdv	*_ppvar[6]._pval
#define caisoma	*_ppvar[7]._pval
#define _p_caisoma	_ppvar[7]._pval
#define nmdasyn	*_ppvar[8]._pval
#define _p_nmdasyn	_ppvar[8]._pval
 
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
 static int hoc_nrnpointerindex =  7;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
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
 "setdata_nmda", _hoc_setdata,
 0, 0
};
 /* declare global and static user variables */
#define cao cao_nmda
 double cao = 2;
#define ko ko_nmda
 double ko = 2.5;
#define ki ki_nmda
 double ki = 140;
#define lamda lamda_nmda
 double lamda = 0.75;
#define lamdaca lamdaca_nmda
 double lamdaca = 0.3;
#define nao nao_nmda
 double nao = 145;
#define pinf pinf_nmda
 double pinf = 0;
#define q q_nmda
 double q = 9;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "cao_nmda", "mM",
 "nao_nmda", "mM",
 "ki_nmda", "mM",
 "ko_nmda", "mM",
 "q_nmda", "mV",
 "Pbar_nmda", "cm/s",
 "km_nmda", "mM",
 "mg_nmda", "mM",
 "ica_nmda", "mA/cm2",
 "ina_nmda", "mA/cm2",
 "ik_nmda", "mA/cm2",
 "inmda_nmda", "mA/cm2",
 "caisoma_nmda", "mM",
 "nmdasyn_nmda", "1",
 0,0
};
 static double delta_t = 1;
 static double p0 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "cao_nmda", &cao_nmda,
 "lamdaca_nmda", &lamdaca_nmda,
 "lamda_nmda", &lamda_nmda,
 "nao_nmda", &nao_nmda,
 "ki_nmda", &ki_nmda,
 "ko_nmda", &ko_nmda,
 "q_nmda", &q_nmda,
 "pinf_nmda", &pinf_nmda,
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
 
#define _cvode_ieq _ppvar[9]._i
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"nmda",
 "Pbar_nmda",
 "pr_nmda",
 "km_nmda",
 "mg_nmda",
 0,
 "ica_nmda",
 "ina_nmda",
 "ik_nmda",
 "inmda_nmda",
 0,
 "p_nmda",
 0,
 "caisoma_nmda",
 "nmdasyn_nmda",
 0};
 static Symbol* _ca_sym;
 static Symbol* _na_sym;
 static Symbol* _k_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 12, _prop);
 	/*initialize range parameters*/
 	Pbar = 2.3e-07;
 	pr = 0.0225;
 	km = 50.7;
 	mg = 1.2;
 	_prop->param = _p;
 	_prop->param_size = 12;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 10, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ca_sym);
 	_ppvar[0]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[1]._pval = &prop_ion->param[4]; /* _ion_dicadv */
 prop_ion = need_memb(_na_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[2]._pval = &prop_ion->param[1]; /* nai */
 	_ppvar[3]._pval = &prop_ion->param[3]; /* ina */
 	_ppvar[4]._pval = &prop_ion->param[4]; /* _ion_dinadv */
 prop_ion = need_memb(_k_sym);
 	_ppvar[5]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[6]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 "p_nmda", 0.0001,
 0,0
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*f)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _nmdasyn_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("ca", -10000.);
 	ion_reg("na", -10000.);
 	ion_reg("k", -10000.);
 	_ca_sym = hoc_lookup("ca_ion");
 	_na_sym = hoc_lookup("na_ion");
 	_k_sym = hoc_lookup("k_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 0);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
  hoc_register_prop_size(_mechtype, 12, 10);
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 nmda /home/hbpschool2016/Documents/cell_optimization/x86_64/nmdasyn.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 96485.3;
 static double R = 8.31342;
 static double _zarg , _zpower , _znumerna , _znumerk , _znumerca , _zdenom , _zdenom2 ;
static int _reset;
static char *modelname = "NMDA receptor as a distributed mechanism";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[1], _dlist1[1];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   pinf = pr + ( 1.0 - pr ) / ( 1.0 + ( mg / km ) * exp ( - v / q ) ) ;
   Dp = pinf - p ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 pinf = pr + ( 1.0 - pr ) / ( 1.0 + ( mg / km ) * exp ( - v / q ) ) ;
 Dp = Dp  / (1. - dt*( ( - 1.0 ) )) ;
 return 0;
}
 /*END CVODE*/
 static int states () {_reset=0;
 {
   pinf = pr + ( 1.0 - pr ) / ( 1.0 + ( mg / km ) * exp ( - v / q ) ) ;
    p = p + (1. - exp(dt*(( - 1.0 ))))*(- ( pinf ) / ( ( - 1.0) ) - p) ;
   }
  return 0;
}
 
static int _ode_count(int _type){ return 1;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  nai = _ion_nai;
     _ode_spec1 ();
    }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 1; ++_i) {
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
  nai = _ion_nai;
 _ode_matsol1 ();
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 0, 3);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 1, 4);
   nrn_update_ion_pointer(_na_sym, _ppvar, 2, 1);
   nrn_update_ion_pointer(_na_sym, _ppvar, 3, 3);
   nrn_update_ion_pointer(_na_sym, _ppvar, 4, 4);
   nrn_update_ion_pointer(_k_sym, _ppvar, 5, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 6, 4);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  p = p0;
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
  nai = _ion_nai;
 initmodel();
   }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   pinf = pr + ( 1.0 - pr ) / ( 1.0 + ( mg / km ) * exp ( - v / q ) ) ;
   _zarg = v * FARADAY / ( ( 1000.0 ) * R * ( celsius + 273.15 ) ) ;
   _zpower = nmdasyn * Pbar * ( 0.000001 ) * v * p * FARADAY * FARADAY / ( R * ( celsius + 273.15 ) ) ;
   _znumerna = lamda * nai - lamda * nao * exp ( - _zarg ) ;
   _znumerk = lamda * ki - lamda * ko * exp ( - _zarg ) ;
   _znumerca = caisoma - lamdaca * cao * exp ( - 2.0 * _zarg ) ;
   _zdenom = 1.0 - exp ( - _zarg ) ;
   _zdenom2 = 1.0 - exp ( - 2.0 * _zarg ) ;
   ina = _zpower * _znumerna / _zdenom ;
   ik = _zpower * _znumerk / _zdenom ;
   ica = 10.6 * _zpower * _znumerca / _zdenom2 ;
   inmda = ina + ik + ica ;
   }
 _current += ica;
 _current += ina;
 _current += ik;

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
  nai = _ion_nai;
 _g = _nrn_current(_v + .001);
 	{ double _dik;
 double _dina;
 double _dica;
  _dica = ica;
  _dina = ina;
  _dik = ik;
 _rhs = _nrn_current(_v);
  _ion_dicadv += (_dica - ica)/.001 ;
  _ion_dinadv += (_dina - ina)/.001 ;
  _ion_dikdv += (_dik - ik)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ica += ica ;
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
  nai = _ion_nai;
 { {
 for (; t < _break; t += dt) {
 error =  states();
 if(error){fprintf(stderr,"at line 80 in file nmdasyn.mod:\n     SOLVE states METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
 
}}
 t = _save;
 }   }}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(p) - _p;  _dlist1[0] = &(Dp) - _p;
_first = 0;
}
