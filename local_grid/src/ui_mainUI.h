/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QVBoxLayout *verticalLayout;
    QSplitter *splitter;
    QFrame *beta_frame;
    QFrame *metrics_frame;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QString::fromUtf8("guiDlg"));
        guiDlg->resize(769, 587);
        verticalLayout = new QVBoxLayout(guiDlg);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        splitter = new QSplitter(guiDlg);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Vertical);
        beta_frame = new QFrame(splitter);
        beta_frame->setObjectName(QString::fromUtf8("beta_frame"));
        beta_frame->setFrameShape(QFrame::StyledPanel);
        beta_frame->setFrameShadow(QFrame::Raised);
        splitter->addWidget(beta_frame);
        metrics_frame = new QFrame(splitter);
        metrics_frame->setObjectName(QString::fromUtf8("metrics_frame"));
        metrics_frame->setFrameShape(QFrame::StyledPanel);
        metrics_frame->setFrameShadow(QFrame::Raised);
        splitter->addWidget(metrics_frame);

        verticalLayout->addWidget(splitter);


        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "giraff_viewer", nullptr));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
