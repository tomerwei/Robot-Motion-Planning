/****************************************************************************
** Meta object code from reading C++ file 'GraphicsEllipse.h'
**
** Created: Sun 12. May 15:50:29 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../GraphicsEllipse.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'GraphicsEllipse.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_GraphicsEllipse[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       1,   24, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      23,   17,   16,   16, 0x09,
      54,   17,   16,   16, 0x09,

 // properties: name, type, flags
      84,   77, 0x14095003,

       0        // eod
};

static const char qt_meta_stringdata_GraphicsEllipse[] = {
    "GraphicsEllipse\0\0event\0"
    "dragMoveEvent(QDragMoveEvent*)\0"
    "dropEvent(QDropEvent*)\0QRectF\0Rect\0"
};

void GraphicsEllipse::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        GraphicsEllipse *_t = static_cast<GraphicsEllipse *>(_o);
        switch (_id) {
        case 0: _t->dragMoveEvent((*reinterpret_cast< QDragMoveEvent*(*)>(_a[1]))); break;
        case 1: _t->dropEvent((*reinterpret_cast< QDropEvent*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData GraphicsEllipse::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject GraphicsEllipse::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_GraphicsEllipse,
      qt_meta_data_GraphicsEllipse, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &GraphicsEllipse::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *GraphicsEllipse::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *GraphicsEllipse::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_GraphicsEllipse))
        return static_cast<void*>(const_cast< GraphicsEllipse*>(this));
    if (!strcmp(_clname, "QGraphicsEllipseItem"))
        return static_cast< QGraphicsEllipseItem*>(const_cast< GraphicsEllipse*>(this));
    return QObject::qt_metacast(_clname);
}

int GraphicsEllipse::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
#ifndef QT_NO_PROPERTIES
      else if (_c == QMetaObject::ReadProperty) {
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QRectF*>(_v) = ellipseProperty(); break;
        }
        _id -= 1;
    } else if (_c == QMetaObject::WriteProperty) {
        void *_v = _a[0];
        switch (_id) {
        case 0: setEllipseProperty(*reinterpret_cast< QRectF*>(_v)); break;
        }
        _id -= 1;
    } else if (_c == QMetaObject::ResetProperty) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 1;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 1;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}
QT_END_MOC_NAMESPACE
