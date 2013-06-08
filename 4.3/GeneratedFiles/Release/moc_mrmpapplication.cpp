/****************************************************************************
** Meta object code from reading C++ file 'mrmpapplication.h'
**
** Created: Sat 8. Jun 17:32:30 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../mrmpapplication.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mrmpapplication.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MRMPApplication[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      17,   16,   16,   16, 0x0a,
      34,   16,   16,   16, 0x0a,
      51,   16,   16,   16, 0x0a,
      64,   16,   16,   16, 0x0a,
      77,   16,   16,   16, 0x0a,
      90,   16,   16,   16, 0x0a,
     103,   16,   16,   16, 0x0a,
     115,   16,   16,   16, 0x0a,
     127,   16,   16,   16, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_MRMPApplication[] = {
    "MRMPApplication\0\0save_workspace()\0"
    "load_workspace()\0save_robot()\0"
    "load_robot()\0save_query()\0load_query()\0"
    "save_path()\0load_path()\0executeComplete()\0"
};

void MRMPApplication::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MRMPApplication *_t = static_cast<MRMPApplication *>(_o);
        switch (_id) {
        case 0: _t->save_workspace(); break;
        case 1: _t->load_workspace(); break;
        case 2: _t->save_robot(); break;
        case 3: _t->load_robot(); break;
        case 4: _t->save_query(); break;
        case 5: _t->load_query(); break;
        case 6: _t->save_path(); break;
        case 7: _t->load_path(); break;
        case 8: _t->executeComplete(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData MRMPApplication::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MRMPApplication::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MRMPApplication,
      qt_meta_data_MRMPApplication, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MRMPApplication::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MRMPApplication::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MRMPApplication::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MRMPApplication))
        return static_cast<void*>(const_cast< MRMPApplication*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MRMPApplication::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
