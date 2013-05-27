/****************************************************************************
** Meta object code from reading C++ file 'Scene.h'
**
** Created: Sun 12. May 15:50:25 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../Scene.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Scene.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Scene[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      23,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
       7,    6,    6,    6, 0x05,
      24,    6,    6,    6, 0x05,

 // slots: signature, parameters, type, tag, flags
      42,   40,    6,    6, 0x0a,
      72,    6,    6,    6, 0x0a,
     103,   89,    6,    6, 0x0a,
     127,   89,    6,    6, 0x0a,
     151,   89,    6,    6, 0x0a,
     179,   89,    6,    6, 0x0a,
     240,  224,  207,    6, 0x0a,
     272,   89,    6,    6, 0x0a,
     296,   89,    6,    6, 0x0a,
     320,   89,    6,    6, 0x0a,
     343,   89,    6,    6, 0x0a,
     366,    6,    6,    6, 0x0a,
     396,  386,    6,    6, 0x0a,
     427,    6,    6,    6, 0x0a,
     464,    6,  445,    6, 0x0a,
     487,    6,  479,    6, 0x0a,
     504,    6,  479,    6, 0x0a,
     552,    6,  525,    6, 0x0a,
     572,    6,  525,    6, 0x0a,
     611,    6,  593,    6, 0x0a,
     635,  630,    6,    6, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Scene[] = {
    "Scene\0\0displayChanged()\0animationDone()\0"
    "e\0mousePressEvent(QMouseEvent*)\0"
    "selectedUpdate()\0pi_szFileName\0"
    "save_robot(const char*)\0load_robot(const char*)\0"
    "save_workspace(const char*)\0"
    "load_workspace(const char*)\0"
    "QVector<QPointF>\0num_vertices,in\0"
    "read_polygon(int,std::istream*)\0"
    "save_query(const char*)\0load_query(const char*)\0"
    "load_path(const char*)\0save_path(const char*)\0"
    "animationComplete()\0file,poly\0"
    "write_polygon(FILE*,QPolygonF)\0"
    "clear_animation()\0QVector<QPolygonF>\0"
    "getObstacles()\0QPointF\0getRoomTopLeft()\0"
    "getRoomBottomRight()\0QVector<QVector<QPointF> >\0"
    "getStartPositions()\0getTargetPositions()\0"
    "vector<QPolygonF>\0getRobotPolygons()\0"
    "path\0setPath(vector<vector<vector<QConf> > >)\0"
};

void Scene::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Scene *_t = static_cast<Scene *>(_o);
        switch (_id) {
        case 0: _t->displayChanged(); break;
        case 1: _t->animationDone(); break;
        case 2: _t->mousePressEvent((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 3: _t->selectedUpdate(); break;
        case 4: _t->save_robot((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 5: _t->load_robot((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 6: _t->save_workspace((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 7: _t->load_workspace((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 8: { QVector<QPointF> _r = _t->read_polygon((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< std::istream*(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< QVector<QPointF>*>(_a[0]) = _r; }  break;
        case 9: _t->save_query((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 10: _t->load_query((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 11: _t->load_path((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 12: _t->save_path((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 13: _t->animationComplete(); break;
        case 14: _t->write_polygon((*reinterpret_cast< FILE*(*)>(_a[1])),(*reinterpret_cast< QPolygonF(*)>(_a[2]))); break;
        case 15: _t->clear_animation(); break;
        case 16: { QVector<QPolygonF> _r = _t->getObstacles();
            if (_a[0]) *reinterpret_cast< QVector<QPolygonF>*>(_a[0]) = _r; }  break;
        case 17: { QPointF _r = _t->getRoomTopLeft();
            if (_a[0]) *reinterpret_cast< QPointF*>(_a[0]) = _r; }  break;
        case 18: { QPointF _r = _t->getRoomBottomRight();
            if (_a[0]) *reinterpret_cast< QPointF*>(_a[0]) = _r; }  break;
        case 19: { QVector<QVector<QPointF> > _r = _t->getStartPositions();
            if (_a[0]) *reinterpret_cast< QVector<QVector<QPointF> >*>(_a[0]) = _r; }  break;
        case 20: { QVector<QVector<QPointF> > _r = _t->getTargetPositions();
            if (_a[0]) *reinterpret_cast< QVector<QVector<QPointF> >*>(_a[0]) = _r; }  break;
        case 21: { vector<QPolygonF> _r = _t->getRobotPolygons();
            if (_a[0]) *reinterpret_cast< vector<QPolygonF>*>(_a[0]) = _r; }  break;
        case 22: _t->setPath((*reinterpret_cast< vector<vector<vector<QConf> > >(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Scene::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Scene::staticMetaObject = {
    { &QGraphicsScene::staticMetaObject, qt_meta_stringdata_Scene,
      qt_meta_data_Scene, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Scene::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Scene::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Scene::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Scene))
        return static_cast<void*>(const_cast< Scene*>(this));
    return QGraphicsScene::qt_metacast(_clname);
}

int Scene::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGraphicsScene::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 23)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 23;
    }
    return _id;
}

// SIGNAL 0
void Scene::displayChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void Scene::animationDone()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
QT_END_MOC_NAMESPACE
