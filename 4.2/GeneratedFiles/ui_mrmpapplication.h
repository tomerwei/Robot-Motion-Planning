/********************************************************************************
** Form generated from reading UI file 'mrmpapplication.ui'
**
** Created: Sat 29. Jun 05:02:39 2013
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MRMPAPPLICATION_H
#define UI_MRMPAPPLICATION_H

#include <MRMPDrawingBox.h>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MRMPApplicationClass
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout_2;
    QGroupBox *groupBox;
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QPushButton *buttonDrawObstacles_3;
    QPushButton *buttonDrawRobots_3;
    QPushButton *buttonAnimate;
    QPushButton *buttonExecuteMP_3;
    QPushButton *buttonAddConfiguration;
    QPushButton *buttonLoadWorkspace;
    QPushButton *buttonSaveWorkspace;
    QPushButton *buttonLoadRobot;
    QPushButton *buttonSaveRobot;
    QPushButton *buttonLoadQuery;
    QPushButton *buttonSaveQuery;
    QPushButton *buttonLoadPath;
    QPushButton *buttonSavePath;
    MRMPDrawingBox *DrawingBox;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MRMPApplicationClass)
    {
        if (MRMPApplicationClass->objectName().isEmpty())
            MRMPApplicationClass->setObjectName(QString::fromUtf8("MRMPApplicationClass"));
        MRMPApplicationClass->resize(817, 614);
        QIcon icon;
        icon.addFile(QString::fromUtf8("Resources/icons/robot4_32.png"), QSize(), QIcon::Normal, QIcon::Off);
        MRMPApplicationClass->setWindowIcon(icon);
        MRMPApplicationClass->setWindowOpacity(1);
        MRMPApplicationClass->setToolButtonStyle(Qt::ToolButtonIconOnly);
        centralWidget = new QWidget(MRMPApplicationClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setMouseTracking(false);
        centralWidget->setAutoFillBackground(false);
        gridLayout_2 = new QGridLayout(centralWidget);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy);
        groupBox->setMinimumSize(QSize(251, 521));
        groupBox->setMaximumSize(QSize(251, 16777215));
        groupBox->setFlat(false);
        groupBox->setCheckable(false);
        layoutWidget = new QWidget(groupBox);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 20, 231, 311));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setHorizontalSpacing(12);
        gridLayout->setContentsMargins(0, 0, 0, 0);
        buttonDrawObstacles_3 = new QPushButton(layoutWidget);
        buttonDrawObstacles_3->setObjectName(QString::fromUtf8("buttonDrawObstacles_3"));

        gridLayout->addWidget(buttonDrawObstacles_3, 0, 0, 1, 1);

        buttonDrawRobots_3 = new QPushButton(layoutWidget);
        buttonDrawRobots_3->setObjectName(QString::fromUtf8("buttonDrawRobots_3"));

        gridLayout->addWidget(buttonDrawRobots_3, 0, 1, 1, 1);

        buttonAnimate = new QPushButton(layoutWidget);
        buttonAnimate->setObjectName(QString::fromUtf8("buttonAnimate"));
        buttonAnimate->setEnabled(true);

        gridLayout->addWidget(buttonAnimate, 4, 0, 1, 1);

        buttonExecuteMP_3 = new QPushButton(layoutWidget);
        buttonExecuteMP_3->setObjectName(QString::fromUtf8("buttonExecuteMP_3"));
        buttonExecuteMP_3->setEnabled(true);

        gridLayout->addWidget(buttonExecuteMP_3, 4, 1, 1, 1);

        buttonAddConfiguration = new QPushButton(layoutWidget);
        buttonAddConfiguration->setObjectName(QString::fromUtf8("buttonAddConfiguration"));

        gridLayout->addWidget(buttonAddConfiguration, 5, 1, 1, 1);

        buttonLoadWorkspace = new QPushButton(layoutWidget);
        buttonLoadWorkspace->setObjectName(QString::fromUtf8("buttonLoadWorkspace"));

        gridLayout->addWidget(buttonLoadWorkspace, 7, 1, 1, 1);

        buttonSaveWorkspace = new QPushButton(layoutWidget);
        buttonSaveWorkspace->setObjectName(QString::fromUtf8("buttonSaveWorkspace"));

        gridLayout->addWidget(buttonSaveWorkspace, 7, 0, 1, 1);

        buttonLoadRobot = new QPushButton(layoutWidget);
        buttonLoadRobot->setObjectName(QString::fromUtf8("buttonLoadRobot"));

        gridLayout->addWidget(buttonLoadRobot, 8, 1, 1, 1);

        buttonSaveRobot = new QPushButton(layoutWidget);
        buttonSaveRobot->setObjectName(QString::fromUtf8("buttonSaveRobot"));

        gridLayout->addWidget(buttonSaveRobot, 8, 0, 1, 1);

        buttonLoadQuery = new QPushButton(layoutWidget);
        buttonLoadQuery->setObjectName(QString::fromUtf8("buttonLoadQuery"));

        gridLayout->addWidget(buttonLoadQuery, 9, 1, 1, 1);

        buttonSaveQuery = new QPushButton(layoutWidget);
        buttonSaveQuery->setObjectName(QString::fromUtf8("buttonSaveQuery"));

        gridLayout->addWidget(buttonSaveQuery, 9, 0, 1, 1);

        buttonLoadPath = new QPushButton(layoutWidget);
        buttonLoadPath->setObjectName(QString::fromUtf8("buttonLoadPath"));

        gridLayout->addWidget(buttonLoadPath, 10, 1, 1, 1);

        buttonSavePath = new QPushButton(layoutWidget);
        buttonSavePath->setObjectName(QString::fromUtf8("buttonSavePath"));

        gridLayout->addWidget(buttonSavePath, 10, 0, 1, 1);


        gridLayout_2->addWidget(groupBox, 0, 1, 1, 1);

        DrawingBox = new MRMPDrawingBox(centralWidget);
        DrawingBox->setObjectName(QString::fromUtf8("DrawingBox"));
        DrawingBox->setEnabled(true);
        DrawingBox->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        DrawingBox->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        DrawingBox->setDragMode(QGraphicsView::RubberBandDrag);

        gridLayout_2->addWidget(DrawingBox, 0, 0, 1, 1);

        MRMPApplicationClass->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MRMPApplicationClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        mainToolBar->setEnabled(true);
        MRMPApplicationClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MRMPApplicationClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MRMPApplicationClass->setStatusBar(statusBar);

        mainToolBar->addSeparator();

        retranslateUi(MRMPApplicationClass);
        QObject::connect(buttonDrawObstacles_3, SIGNAL(clicked()), DrawingBox, SLOT(drawObstaclesButtonPushed()));
        QObject::connect(DrawingBox, SIGNAL(plannerFinished()), MRMPApplicationClass, SLOT(executeComplete()));
        QObject::connect(buttonDrawRobots_3, SIGNAL(clicked()), DrawingBox, SLOT(drawRobotsButtonPushed()));
        QObject::connect(buttonAddConfiguration, SIGNAL(clicked()), DrawingBox, SLOT(addConfigurationButtonPressed()));
        QObject::connect(buttonAnimate, SIGNAL(clicked()), DrawingBox, SLOT(animateButtonPressed()));
        QObject::connect(buttonLoadWorkspace, SIGNAL(clicked()), MRMPApplicationClass, SLOT(load_workspace()));
        QObject::connect(buttonSaveWorkspace, SIGNAL(clicked()), MRMPApplicationClass, SLOT(save_workspace()));
        QObject::connect(buttonLoadRobot, SIGNAL(clicked()), MRMPApplicationClass, SLOT(load_robot()));
        QObject::connect(buttonSaveRobot, SIGNAL(clicked()), MRMPApplicationClass, SLOT(save_robot()));
        QObject::connect(buttonSaveQuery, SIGNAL(clicked()), MRMPApplicationClass, SLOT(save_query()));
        QObject::connect(buttonLoadQuery, SIGNAL(clicked()), MRMPApplicationClass, SLOT(load_query()));
        QObject::connect(buttonLoadPath, SIGNAL(clicked()), MRMPApplicationClass, SLOT(load_path()));
        QObject::connect(buttonSavePath, SIGNAL(clicked()), MRMPApplicationClass, SLOT(save_path()));
        QObject::connect(buttonExecuteMP_3, SIGNAL(clicked()), DrawingBox, SLOT(execute()));

        QMetaObject::connectSlotsByName(MRMPApplicationClass);
    } // setupUi

    void retranslateUi(QMainWindow *MRMPApplicationClass)
    {
        MRMPApplicationClass->setWindowTitle(QApplication::translate("MRMPApplicationClass", "Multiple Robot Motion Planner ", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MRMPApplicationClass", "Controls", 0, QApplication::UnicodeUTF8));
        buttonDrawObstacles_3->setText(QApplication::translate("MRMPApplicationClass", "DrawObstacles", 0, QApplication::UnicodeUTF8));
        buttonDrawRobots_3->setText(QApplication::translate("MRMPApplicationClass", "DrawRobots", 0, QApplication::UnicodeUTF8));
        buttonAnimate->setText(QApplication::translate("MRMPApplicationClass", "Animate", 0, QApplication::UnicodeUTF8));
        buttonExecuteMP_3->setText(QApplication::translate("MRMPApplicationClass", "Execute Motion Planning", 0, QApplication::UnicodeUTF8));
        buttonAddConfiguration->setText(QApplication::translate("MRMPApplicationClass", "Add Configuration", 0, QApplication::UnicodeUTF8));
        buttonLoadWorkspace->setText(QApplication::translate("MRMPApplicationClass", "Load Workspace", 0, QApplication::UnicodeUTF8));
        buttonSaveWorkspace->setText(QApplication::translate("MRMPApplicationClass", "Save Workspace", 0, QApplication::UnicodeUTF8));
        buttonLoadRobot->setText(QApplication::translate("MRMPApplicationClass", "Load Robot", 0, QApplication::UnicodeUTF8));
        buttonSaveRobot->setText(QApplication::translate("MRMPApplicationClass", "Save Robot", 0, QApplication::UnicodeUTF8));
        buttonLoadQuery->setText(QApplication::translate("MRMPApplicationClass", "Load Query", 0, QApplication::UnicodeUTF8));
        buttonSaveQuery->setText(QApplication::translate("MRMPApplicationClass", "Save Query", 0, QApplication::UnicodeUTF8));
        buttonLoadPath->setText(QApplication::translate("MRMPApplicationClass", "Load Path", 0, QApplication::UnicodeUTF8));
        buttonSavePath->setText(QApplication::translate("MRMPApplicationClass", "Save Path", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MRMPApplicationClass: public Ui_MRMPApplicationClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MRMPAPPLICATION_H
