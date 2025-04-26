/****************************************************************************
** Meta object code from reading C++ file 'Scalar_punk_widget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../Scalar_punk_widget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Scalar_punk_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_RobotEyes_t {
    QByteArrayData data[11];
    char stringdata0[102];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RobotEyes_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RobotEyes_t qt_meta_stringdata_RobotEyes = {
    {
QT_MOC_LITERAL(0, 0, 9), // "RobotEyes"
QT_MOC_LITERAL(1, 10, 15), // "updateAnimation"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 10), // "startBlink"
QT_MOC_LITERAL(4, 38, 12), // "runHappyEyes"
QT_MOC_LITERAL(5, 51, 8), // "lookLeft"
QT_MOC_LITERAL(6, 60, 9), // "lookRight"
QT_MOC_LITERAL(7, 70, 6), // "wakeUp"
QT_MOC_LITERAL(8, 77, 9), // "goToSleep"
QT_MOC_LITERAL(9, 87, 5), // "Angry"
QT_MOC_LITERAL(10, 93, 8) // "Charging"

    },
    "RobotEyes\0updateAnimation\0\0startBlink\0"
    "runHappyEyes\0lookLeft\0lookRight\0wakeUp\0"
    "goToSleep\0Angry\0Charging"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RobotEyes[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   59,    2, 0x08 /* Private */,
       3,    0,   60,    2, 0x08 /* Private */,
       4,    0,   61,    2, 0x08 /* Private */,
       5,    0,   62,    2, 0x08 /* Private */,
       6,    0,   63,    2, 0x08 /* Private */,
       7,    0,   64,    2, 0x08 /* Private */,
       8,    0,   65,    2, 0x08 /* Private */,
       9,    0,   66,    2, 0x08 /* Private */,
      10,    0,   67,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void RobotEyes::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<RobotEyes *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updateAnimation(); break;
        case 1: _t->startBlink(); break;
        case 2: _t->runHappyEyes(); break;
        case 3: _t->lookLeft(); break;
        case 4: _t->lookRight(); break;
        case 5: _t->wakeUp(); break;
        case 6: _t->goToSleep(); break;
        case 7: _t->Angry(); break;
        case 8: _t->Charging(); break;
        default: ;
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject RobotEyes::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_RobotEyes.data,
    qt_meta_data_RobotEyes,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *RobotEyes::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RobotEyes::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_RobotEyes.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int RobotEyes::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
