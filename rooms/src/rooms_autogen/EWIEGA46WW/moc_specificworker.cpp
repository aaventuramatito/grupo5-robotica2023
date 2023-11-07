/****************************************************************************
** Meta object code from reading C++ file 'specificworker.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../specificworker.h"
#include <QtGui/qtextcursor.h>
#include <QScreen>
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'specificworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SpecificWorker_t {
    const uint offsetsAndSize[26];
    char stringdata0[142];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_SpecificWorker_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_SpecificWorker_t qt_meta_stringdata_SpecificWorker = {
    {
QT_MOC_LITERAL(0, 14), // "SpecificWorker"
QT_MOC_LITERAL(15, 7), // "compute"
QT_MOC_LITERAL(23, 0), // ""
QT_MOC_LITERAL(24, 13), // "startup_check"
QT_MOC_LITERAL(38, 10), // "initialize"
QT_MOC_LITERAL(49, 6), // "period"
QT_MOC_LITERAL(56, 13), // "straight_line"
QT_MOC_LITERAL(70, 25), // "RoboCompLidar3D::TPoints&"
QT_MOC_LITERAL(96, 15), // "filtered_points"
QT_MOC_LITERAL(112, 11), // "follow_wall"
QT_MOC_LITERAL(124, 6), // "spiral"
QT_MOC_LITERAL(131, 4), // "turn"
QT_MOC_LITERAL(136, 5) // "midle"

    },
    "SpecificWorker\0compute\0\0startup_check\0"
    "initialize\0period\0straight_line\0"
    "RoboCompLidar3D::TPoints&\0filtered_points\0"
    "follow_wall\0spiral\0turn\0midle"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SpecificWorker[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   62,    2, 0x0a,    1 /* Public */,
       3,    0,   63,    2, 0x0a,    2 /* Public */,
       4,    1,   64,    2, 0x0a,    3 /* Public */,
       6,    1,   67,    2, 0x0a,    5 /* Public */,
       9,    1,   70,    2, 0x0a,    7 /* Public */,
      10,    1,   73,    2, 0x0a,    9 /* Public */,
      11,    1,   76,    2, 0x0a,   11 /* Public */,
      12,    1,   79,    2, 0x0a,   13 /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Int,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 7,    8,

       0        // eod
};

void SpecificWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SpecificWorker *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->compute(); break;
        case 1: { int _r = _t->startup_check();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 2: _t->initialize((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 3: _t->straight_line((*reinterpret_cast< std::add_pointer_t<RoboCompLidar3D::TPoints&>>(_a[1]))); break;
        case 4: _t->follow_wall((*reinterpret_cast< std::add_pointer_t<RoboCompLidar3D::TPoints&>>(_a[1]))); break;
        case 5: _t->spiral((*reinterpret_cast< std::add_pointer_t<RoboCompLidar3D::TPoints&>>(_a[1]))); break;
        case 6: _t->turn((*reinterpret_cast< std::add_pointer_t<RoboCompLidar3D::TPoints&>>(_a[1]))); break;
        case 7: _t->midle((*reinterpret_cast< std::add_pointer_t<RoboCompLidar3D::TPoints&>>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject SpecificWorker::staticMetaObject = { {
    QMetaObject::SuperData::link<GenericWorker::staticMetaObject>(),
    qt_meta_stringdata_SpecificWorker.offsetsAndSize,
    qt_meta_data_SpecificWorker,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_SpecificWorker_t
, QtPrivate::TypeAndForceComplete<SpecificWorker, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<RoboCompLidar3D::TPoints &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<RoboCompLidar3D::TPoints &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<RoboCompLidar3D::TPoints &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<RoboCompLidar3D::TPoints &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<RoboCompLidar3D::TPoints &, std::false_type>


>,
    nullptr
} };


const QMetaObject *SpecificWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SpecificWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SpecificWorker.stringdata0))
        return static_cast<void*>(this);
    return GenericWorker::qt_metacast(_clname);
}

int SpecificWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GenericWorker::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 8;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
