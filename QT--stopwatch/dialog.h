#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QTime>

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

private:
    Ui::Dialog *ui;


    int state;  //秒表工作状态
    int nMSCnt; //秒表计时(ms)
    int fdigit;     //表盘和数字切换标志
    QTime sTime;    //秒表开始时间
    QList<int> lstCnt;
    QList<int> lstCntd;


    QRect rc;
    QRect rcDigit1;
    QRect rcDigit2;
    QRect rcDigit3;
    QRect rcDigit4;
    QRect rctmp;
    QRect rcLeft;
    QRect rcRight;
    QRect rcDown;

    QPoint poEllipse;

    //其他变量
    QString str1;
    QString str2;
    QString str3;
    int nms;
    int nnms;
    int nnnms;
    int idx;
    int maxidx,maxnms;
    int minidx,minnms;

private slots:
    void mytimer();


    // QWidget interface
protected:
    void paintEvent(QPaintEvent *);

    // QWidget interface
protected:
    void mousePressEvent(QMouseEvent *);

    // QWidget interface
protected:
    void wheelEvent(QWheelEvent *);
};

#endif // DIALOG_H
