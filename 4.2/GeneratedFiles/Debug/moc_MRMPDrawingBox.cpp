/****************************************************************************
** Meta object code from reading C++ file 'MRMPDrawingBox.h'
**
** Created: Mon 27. May 16:23:31 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../MRMPDrawingBox.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MRMPDrawingBox.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MRMPDrawingBox[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x05,

 // slots: signature, parameters, type, tag, flags
      34,   15,   15,   15, 0x0a,
      57,   15,   15,   15, 0x0a,
      89,   15,   15,   15, 0x0a,
      99,   15,   15,   15, 0x0a,
     121,  119,   15,   15, 0x0a,
     151,   15,   15,   15, 0x0a,
     179,   15,   15,   15, 0x0a,
     204,   15,   15,   15, 0x0a,
     212,   15,   15,   15, 0x0a,
     243,  227,   15,   15, 0x0a,
     271,   15,   15,   15, 0x0a,
     283,  281,   15,   15, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_MRMPDrawingBox[] = {
    "MRMPDrawingBox\0\0plannerFinished()\0"
    "animateButtonPressed()\0"
    "addConfigurationButtonPressed()\0"
    "refresh()\0animationComplete()\0e\0"
    "mousePressEvent(QMouseEvent*)\0"
    "drawObstaclesButtonPushed()\0"
    "drawRobotsButtonPushed()\0clear()\0"
    "clearResults()\0animationFactor\0"
    "setAnimationMultiplier(int)\0execute()\0"
    "s\0selected_robot_changed(QString)\0"
};

void MRMPDrawingBox::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MRMPDrawingBox *_t = static_cast<MRMPDrawingBox *>(_o);
        switch (_id) {
        case 0: _t->plannerFinished(); break;
        case 1: _t->animateButtonPressed(); break;
        case 2: _t->addConfigurationButtonPressed(); break;
        case 3: _t->refresh(); break;
        case 4: _t->animationComplete(); break;
        case 5: _t->mousePressEvent((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 6: _t->drawObstaclesButtonPushed(); break;
        case 7: _t->drawRobotsButtonPushed(); break;
        case 8: _t->clear(); break;
        case 9: _t->clearResults(); break;
        case 10: _t->setAnimationMultiplier((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->execute(); break;
        case 12: _t->selected_robot_changed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MRMPDrawingBox::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MRMPDrawingBox::staticMetaObject = {
    { &QGraphicsView::staticMetaObject, qt_meta_stringdata_MRMPDrawingBox,
      qt_meta_data_MRMPDrawingBox, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MRMPDrawingBox::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MRMPDrawingBox::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MRMPDrawingBox::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MRMPDrawingBox))
        return static_cast<void*>(const_cast< MRMPDrawingBox*>(this));
    return QGraphicsView::qt_metacast(_clname);
}

int MRMPDrawingBox::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGraphicsView::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void MRMPDrawingBox::plannerFinished()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
