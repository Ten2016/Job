#include "dialog.h"
#include "ui_dialog.h"

#include <QPainter>
#include <QTimer>
#include <QMouseEvent>
#include <QtMath>

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    //窗体标题
    this->setWindowTitle("秒表");
    //窗口大小初始化
    this-> resize( QSize( 400, 700 ));
    //背景黑色
    this->setStyleSheet("background:black");
    //变量初始化

    state =0;
    fdigit=1;
    nMSCnt=0;
    idx=0;
    maxidx=minidx=0;
    maxnms=minnms=0;
    sTime = QTime::currentTime();

    //初始化定时器
    QTimer *timer=new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(mytimer()));
    timer ->start(10);//定时周期10ms

}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::mytimer()
{
    if(1==state){
        update();
    }
}





void Dialog::paintEvent(QPaintEvent *)
{
    QPainter qp(this);
    QPen pen=qp.pen();
    qp.setRenderHint(QPainter::Antialiasing);//反锯齿平滑

    if(state>0){
        if(1==state)
            nms=(sTime.msecsTo(QTime::currentTime())+nMSCnt) / 10;
        else
            nms = nMSCnt /10;
        str1.sprintf("%02d:%02d.%02d",nms/6000,(nms/100)%60,nms%100);
        str2.sprintf("%02d.%02d",nms/6000,(nms%6000)/60);
    }
    else{
        str1="00:00.00";          //秒计时
        str2="00.00";               //分钟计时
    }

    //主窗口
    rc=rect();
    int rcW=rc.width();
    int rcH=rc.height();
    int h2=rcH/2,w2=rcW/2;
    int w15=rcW/15;
    int h25=rcH/25,w20=rcW/20;

    //表盘框
    rcDigit1 = QRect(w15, h25, rcW-w15*2, h2-h25);
    int rcDigit1H=rcDigit1.height();
    int rcDigit1W=rcDigit1.width();
    int r=rcDigit1H > rcDigit1W ? rcDigit1W : rcDigit1H;

    //左边按钮
    rcLeft = QRect(0, h2, w2-w15, rcH/6);
    //右边按钮
    rcRight =QRect(w2+w15, h2, w2-w15, rcH/6);
    int rx=rcLeft.height() > rcLeft.width() ? rcLeft.width() : rcLeft.height();
    rx/=2;
    qp.setPen(Qt::NoPen);       //关闭画笔，使用画刷
    QBrush brush(QColor(40, 40, 40));
    if(state)
        qp.setBrush(brush);
    qp.drawEllipse(rcLeft.center(),rx,rx);
    if(state==1)
        brush.setColor(QColor(40, 0, 0));
    else
        brush.setColor(QColor(0, 40, 0));
    qp.setBrush(brush);
    qp.drawEllipse(rcRight.center(),rx,rx);
    qp.setBrush(Qt::NoBrush);       //关闭画刷
    qp.setFont(QFont("Arial",rx/2));
    switch(state){
    default:
            qp.setPen(Qt::white);
            qp.drawText(rcLeft, Qt::AlignCenter, "计次");
            qp.setPen(Qt::red);
            qp.drawText(rcRight, Qt::AlignCenter, "启动");
        break;
    case 1:
       qp.setPen(Qt::white);
       qp.drawText(rcLeft, Qt::AlignCenter, "计次");
       qp.setPen(Qt::red);
       qp.drawText(rcRight, Qt::AlignCenter, "停止");
        break;
    case 2:
        qp.setPen(Qt::white);
        qp.drawText(rcLeft, Qt::AlignCenter, "复位");
        qp.setPen(Qt::red);
        qp.drawText(rcRight, Qt::AlignCenter, "启动");
        break;
    }

    //开始计数后
    if (lstCnt.empty()==0){
        nnnms =nms-lstCnt.last() /10;
        if(fdigit==-1){
            str3.sprintf("%02d:%02d.%02d",nnnms/6000,(nnnms/100)%60,nnnms%100);
            //数字模式计次框
            rcDigit3 = QRect(w15, h25+rcDigit1H/2+r/5, rcDigit1W, r/7);
            qp.setPen(QColor(128,128,128));
            qp.setFont(QFont("Arial",rcDigit3.height()));
            qp.drawText(rcDigit3,Qt::AlignCenter ,str3);
        }
        //底部计次
        rcDown = QRect(w20, h2+rcH/16+rcH/8, rcW-w20*2, h2-rcH/5-rcH/48);
        if(rcDown.width()>rcDown.height()*2)
            qp.setFont(QFont("Arial",rcDown.height()/8));
        else
            qp.setFont(QFont("Arial",rcDown.width()/16));

        int i=idx;
        if(i>=lstCnt.size())
            i=lstCnt.size() -1;
        //显示最近四次
        for( int j=i-4;i>=0 && i>j; i--){
            nnms = lstCntd[i];
            str3.sprintf("计次  %02d          %02d:%02d.%02d",i+1,
                         nnms/6000, (nnms/100)%60, nnms%100);
            if(i==maxidx)
                qp.setPen(Qt::red);
            else if(i==minidx)
                qp.setPen(Qt::green);
            else
                qp.setPen(Qt::white);
             qp.drawText(rcDown, Qt::AlignTop | Qt::AlignHCenter, str3);
             qp.setPen(Qt::white);
             //绘制计次底部横线
             qp.drawLine(QPoint(rcDown.center().rx()-rcDown.width()/2,
                                rcDown.center().ry()-rcDown.height()/4),
                                    QPoint(rcDown.center().rx()+rcDown.width()/2,
                                rcDown.center().ry()-rcDown.height()/4));
             rcDown.translate(0,rcDown.height() / 4);
        }
    }


//表盘模式
    if(fdigit==1){
        //表盘中心点
        poEllipse = rcDigit1.center();
        int r2=r/2;
        int r5=r/5;
        int r10=r/10;
        int r20=r/20;
        double v=3.1416/180;
        int rr;

//绘制小表盘
        qp.translate(QPoint(poEllipse.rx(), poEllipse.ry()-r5));

        //绘制小表盘数字
        rr=r/12;
        pen.setWidth(r/90);
        pen.setColor(Qt::white);
        qp.setPen(pen);
        qp.setFont(QFont("Arial",r/25));
        for(int i=0;i<6;i++){
            rctmp = QRect(QPoint(rr*qCos(v*(i*60-90))-r/40, rr*qSin(v*(i*60-90))-r/40), QSize(r20,r20));
            int nn=i*5;
            if(nn==0)
                nn=30;
            qp.drawText(rctmp, Qt::AlignCenter, QString::number(nn));
        }
        // 绘制小表盘
       for (int i = 0; i < 30; i++) {
           if(i%5==0){
               pen.setWidth(r/240);
               pen.setColor(QColor(255, 255, 255));
               qp.setPen(pen);
               qp.drawLine(0, r/40-r/7, 0, -r/7);
           }
           else{
               pen.setWidth(r/280);
               pen.setColor(QColor(128, 128, 128));
               qp.setPen(pen);
               qp.drawLine(0, r/60-r/7, 0, -r/7);
           }
           qp.rotate(12.0);
       }
       //绘制指针
       pen.setWidth(r/120);
       pen.setColor(Qt::green);
       qp.setPen(pen);
       qp.drawEllipse(QPoint(0,0), r/100, r/100);
       pen.setColor(QColor(255, 0, 255));
       qp.setPen(pen);
       qp.rotate(nms/500.0);
       qp.drawLine(0, r/30, 0, -r/8);
       qp.resetTransform();


//绘制大表盘
        qp.translate(poEllipse);
        //绘制大表盘数字
        rr=r2-r2/5;
        pen.setWidth(r/90);
        pen.setColor(Qt::white);
        qp.setPen(pen);
        qp.setFont(QFont("Arial",r20));
        for(int i=0;i<12;i++){
            rctmp = QRect(QPoint(rr*qCos(v*((i-3)*30))-r20, rr*qSin(v*((i-3)*30))-r20), QSize(r10,r10));
            int nn=i*5;
            if(nn==0)
                nn=60;
            qp.drawText(rctmp, Qt::AlignCenter, QString::number(nn));
        }
        //绘制大表盘
        for (int i = 0; i < 60; i++) {
            if(i%5==0){
                pen.setWidth(r/90);
                pen.setColor(QColor(255, 255, 255));
                qp.setPen(pen);
                qp.drawLine(0, r20-r2, 0, -r2);
            }
            else{
                pen.setWidth(r/100);
                pen.setColor(QColor(100, 100, 100));
                qp.setPen(pen);
                qp.drawLine(0, r/36-r2, 0, -r2);
            }
            qp.rotate(6.0);
        }
        //绘制指针
        pen.setWidth(r/110);
        pen.setColor(Qt::green);
        qp.setPen(pen);
        qp.drawEllipse(QPoint(0,0), r/70, r/70);
        if (lstCnt.empty()==0){
            pen.setColor(QColor(0, 255, 255));
            qp.setPen(pen);
            qp.rotate((6.0*nnnms/100));
            qp.drawLine(0, r/16, 0, r20-r2);
            qp.resetTransform();
            qp.translate(poEllipse);
        }
        pen.setColor(Qt::yellow);
        qp.setPen(pen);
        qp.rotate(6.0*nms/100);
        qp.drawLine(0, r/16, 0, r20-r2);
        qp.resetTransform();


        //计时框
        rcDigit2 = QRect(w15, h25+rcDigit1H/2+r/8, rcDigit1W, r/13);
        qp.setPen(Qt::white);
        qp.setFont(QFont("Arial",rcDigit2.height()));
        qp.drawText(rcDigit2,Qt::AlignCenter ,str1);
    }

//数字模式
    else{
        //秒表框
        qp.setPen(Qt::white);
        qp.setFont(QFont("Arial",r/5));
        qp.drawText(rcDigit1,Qt::AlignCenter ,str1);
        //数字模式分钟框
        rcDigit4 = QRect(w15, h25+rcDigit1H/2-r/3, rcDigit1W, r/8);
        qp.setPen(QColor(32,32,32));
        qp.setFont(QFont("Arial",rcDigit4.height()));
        qp.drawText(rcDigit4,Qt::AlignCenter ,str2);
    }
}


void Dialog::mousePressEvent(QMouseEvent *e)
{
    if (Qt::LeftButton == e->button() ){
        if (rcDigit1.contains(e->pos())){
            fdigit=-fdigit;
            update();
        }
        switch(state){
            default:
                if (rcRight.contains(e->pos())){
                    nMSCnt =0;
                    sTime = QTime::currentTime();
                    state=1;
                }
                break;
        case 1:
            if(rcLeft.contains(e->pos())){
                if(lstCnt.size()<100){
                    lstCnt.append(nMSCnt + sTime.msecsTo(QTime::currentTime()));
                    idx=lstCnt.size() -1;
                    if(idx==0){
                        lstCntd.append(lstCnt[0]/10);
                        maxidx=minidx=0;
                        maxnms=minnms=lstCntd[0];
                    }
                    else{
                        lstCntd.append((lstCnt[idx]-lstCnt[idx-1])/10);
                        if(lstCntd[idx]>maxnms){
                            maxidx=idx;
                            maxnms=lstCntd[idx];
                        }
                        if(lstCntd[idx]<minnms){
                            minidx=idx;
                            minnms=lstCntd[idx];
                        }
                    }
                    update();
                }
            }
            else if(rcRight.contains(e->pos())){
                state =2;
                nMSCnt += sTime.msecsTo(QTime::currentTime());
                update();
            }
            break;
        case 2:
            if(rcLeft.contains(e->pos())){
                state=0;
                nms=0;
                nnnms=0;
                nMSCnt=0;
                lstCnt.clear();
                lstCntd.clear();
                update();
            }
            else if(rcRight.contains(e->pos())){
                state =1;
                sTime = QTime::currentTime();
            }
            break;
        }
    }
}


void Dialog::wheelEvent(QWheelEvent *e)
{
    if (e->delta()>0){
        if(idx < lstCnt.size() -1){
            idx++;
            update();
        }
    }
    else{
        if(idx>0){
            idx--;
            update();
        }
    }
}
