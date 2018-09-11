/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDockWidget>
#include <QtGui/QFrame>
#include <QtGui/QGraphicsView>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QTabWidget *tab_manager;
    QWidget *tab_status;
    QGraphicsView *graphicsView;
    QLabel *label_2;
    QListView *view_logging;
    QLabel *label_3;
    QStatusBar *statusbar;
    QDockWidget *dock_status;
    QWidget *dockWidgetContents_2;
    QVBoxLayout *verticalLayout;
    QFrame *frame;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupBox;
    QGridLayout *gridLayout;
    QLabel *label;
    QLineEdit *line_edit_master;
    QCheckBox *checkbox_use_environment;
    QSpacerItem *horizontalSpacer;
    QPushButton *button_connect;
    QSpacerItem *verticalSpacer_3;
    QPushButton *quit_button;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(944, 704);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QString::fromUtf8("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QString::fromUtf8("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        tab_manager = new QTabWidget(centralwidget);
        tab_manager->setObjectName(QString::fromUtf8("tab_manager"));
        tab_manager->setMinimumSize(QSize(100, 0));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_status = new QWidget();
        tab_status->setObjectName(QString::fromUtf8("tab_status"));
        graphicsView = new QGraphicsView(tab_status);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setGeometry(QRect(10, 230, 571, 391));
        label_2 = new QLabel(tab_status);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 210, 67, 17));
        view_logging = new QListView(tab_status);
        view_logging->setObjectName(QString::fromUtf8("view_logging"));
        view_logging->setGeometry(QRect(10, 50, 571, 151));
        label_3 = new QLabel(tab_status);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 30, 67, 17));
        tab_manager->addTab(tab_status, QString());

        hboxLayout->addWidget(tab_manager);

        MainWindowDesign->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);
        dock_status = new QDockWidget(MainWindowDesign);
        dock_status->setObjectName(QString::fromUtf8("dock_status"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(dock_status->sizePolicy().hasHeightForWidth());
        dock_status->setSizePolicy(sizePolicy);
        dock_status->setMinimumSize(QSize(325, 389));
        dock_status->setAllowedAreas(Qt::RightDockWidgetArea);
        dockWidgetContents_2 = new QWidget();
        dockWidgetContents_2->setObjectName(QString::fromUtf8("dockWidgetContents_2"));
        verticalLayout = new QVBoxLayout(dockWidgetContents_2);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        frame = new QFrame(dockWidgetContents_2);
        frame->setObjectName(QString::fromUtf8("frame"));
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout_3 = new QVBoxLayout(frame);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        groupBox = new QGroupBox(frame);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout = new QGridLayout(groupBox);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setFrameShape(QFrame::StyledPanel);
        label->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(label, 0, 0, 1, 1);

        line_edit_master = new QLineEdit(groupBox);
        line_edit_master->setObjectName(QString::fromUtf8("line_edit_master"));

        gridLayout->addWidget(line_edit_master, 1, 0, 1, 2);

        checkbox_use_environment = new QCheckBox(groupBox);
        checkbox_use_environment->setObjectName(QString::fromUtf8("checkbox_use_environment"));
        checkbox_use_environment->setLayoutDirection(Qt::RightToLeft);

        gridLayout->addWidget(checkbox_use_environment, 2, 0, 1, 2);

        horizontalSpacer = new QSpacerItem(170, 21, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 3, 0, 1, 1);

        button_connect = new QPushButton(groupBox);
        button_connect->setObjectName(QString::fromUtf8("button_connect"));
        button_connect->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(button_connect->sizePolicy().hasHeightForWidth());
        button_connect->setSizePolicy(sizePolicy1);

        gridLayout->addWidget(button_connect, 3, 1, 1, 1);


        verticalLayout_3->addWidget(groupBox);

        verticalSpacer_3 = new QSpacerItem(20, 233, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_3);


        verticalLayout->addWidget(frame);

        quit_button = new QPushButton(dockWidgetContents_2);
        quit_button->setObjectName(QString::fromUtf8("quit_button"));
        sizePolicy1.setHeightForWidth(quit_button->sizePolicy().hasHeightForWidth());
        quit_button->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(quit_button);

        dock_status->setWidget(dockWidgetContents_2);
        MainWindowDesign->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dock_status);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));
        QObject::connect(quit_button, SIGNAL(clicked()), MainWindowDesign, SLOT(close()));

        tab_manager->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "QRosApp", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindowDesign", "Model", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindowDesign", "Logging", 0, QApplication::UnicodeUTF8));
        tab_manager->setTabText(tab_manager->indexOf(tab_status), QApplication::translate("MainWindowDesign", "Ros Communications", 0, QApplication::UnicodeUTF8));
        dock_status->setWindowTitle(QApplication::translate("MainWindowDesign", "Command Panel", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindowDesign", "Ros Master", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindowDesign", "Ros Master Url", 0, QApplication::UnicodeUTF8));
        line_edit_master->setText(QApplication::translate("MainWindowDesign", "http://192.168.1.2:11311/", 0, QApplication::UnicodeUTF8));
        checkbox_use_environment->setText(QApplication::translate("MainWindowDesign", "Use environment variables", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        button_connect->setToolTip(QApplication::translate("MainWindowDesign", "Set the target to the current joint trajectory state.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        button_connect->setStatusTip(QApplication::translate("MainWindowDesign", "Clear all waypoints and set the target to the current joint trajectory state.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        button_connect->setText(QApplication::translate("MainWindowDesign", "Connect", 0, QApplication::UnicodeUTF8));
        quit_button->setText(QApplication::translate("MainWindowDesign", "Quit", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
