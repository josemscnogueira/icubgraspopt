/****************************************************************************
** Meta object code from reading C++ file 'Window.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Window.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Window.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ShowWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      29,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x0a,
      31,   11,   11,   11, 0x0a,
      47,   11,   11,   11, 0x0a,
      71,   65,   11,   11, 0x0a,
      86,   65,   11,   11, 0x0a,
     108,  103,   11,   11, 0x0a,
     130,  103,   11,   11, 0x0a,
     160,  150,   11,   11, 0x0a,
     187,  183,   11,   11, 0x0a,
     209,  183,   11,   11, 0x0a,
     231,  183,   11,   11, 0x0a,
     253,  183,   11,   11, 0x0a,
     275,  183,   11,   11, 0x0a,
     297,  183,   11,   11, 0x0a,
     319,  183,   11,   11, 0x0a,
     341,  183,   11,   11, 0x0a,
     363,  183,   11,   11, 0x0a,
     385,  183,   11,   11, 0x0a,
     406,  183,   11,   11, 0x0a,
     427,  183,   11,   11, 0x0a,
     448,  183,   11,   11, 0x0a,
     469,  183,   11,   11, 0x0a,
     490,  183,   11,   11, 0x0a,
     511,  183,   11,   11, 0x0a,
     531,   11,   11,   11, 0x0a,
     551,   11,   11,   11, 0x0a,
     574,   11,   11,   11, 0x0a,
     592,   11,   11,   11, 0x0a,
     603,   11,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_ShowWindow[] = {
    "ShowWindow\0\0searchObjectTest()\0"
    "optimizeGrasp()\0graspMetricTest()\0"
    "index\0selectRNS(int)\0selectJoint(int)\0"
    "mode\0selectSliderMode(int)\0"
    "selectMoveHand(int)\0slide_pos\0"
    "jointValueChanged(int)\0pos\0"
    "handJointSlider1(int)\0handJointSlider2(int)\0"
    "handJointSlider3(int)\0handJointSlider4(int)\0"
    "handJointSlider5(int)\0handJointSlider6(int)\0"
    "handJointSlider7(int)\0handJointSlider8(int)\0"
    "handJointSlider9(int)\0moveHandSlider1(int)\0"
    "moveHandSlider2(int)\0moveHandSlider3(int)\0"
    "moveHandSlider4(int)\0moveHandSlider5(int)\0"
    "moveHandSlider6(int)\0selectGraspOpt(int)\0"
    "resetGraspSliders()\0resetMoveHandSliders()\0"
    "resetSceneryAll()\0loadHand()\0closeGrasp()\0"
};

void ShowWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ShowWindow *_t = static_cast<ShowWindow *>(_o);
        switch (_id) {
        case 0: _t->searchObjectTest(); break;
        case 1: _t->optimizeGrasp(); break;
        case 2: _t->graspMetricTest(); break;
        case 3: _t->selectRNS((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->selectJoint((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->selectSliderMode((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->selectMoveHand((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->jointValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->handJointSlider1((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->handJointSlider2((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->handJointSlider3((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->handJointSlider4((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->handJointSlider5((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->handJointSlider6((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->handJointSlider7((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 15: _t->handJointSlider8((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 16: _t->handJointSlider9((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 17: _t->moveHandSlider1((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 18: _t->moveHandSlider2((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 19: _t->moveHandSlider3((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 20: _t->moveHandSlider4((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 21: _t->moveHandSlider5((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 22: _t->moveHandSlider6((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 23: _t->selectGraspOpt((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 24: _t->resetGraspSliders(); break;
        case 25: _t->resetMoveHandSliders(); break;
        case 26: _t->resetSceneryAll(); break;
        case 27: _t->loadHand(); break;
        case 28: _t->closeGrasp(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ShowWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ShowWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_ShowWindow,
      qt_meta_data_ShowWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ShowWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ShowWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ShowWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ShowWindow))
        return static_cast<void*>(const_cast< ShowWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int ShowWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 29)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 29;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
