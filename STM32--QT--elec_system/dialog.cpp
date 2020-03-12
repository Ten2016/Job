#include "dialog.h"
#include "ui_dialog.h"


Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    //查找可用的串口
    ui->comboBox->clear();
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        QSerialPort serial;
        serial.setPort(info);
        if(serial.open(QIODevice::ReadWrite))
        {
            ui->comboBox->addItem(serial.portName());
            serial.close();
        }
    }
    ui->comboBox_2->setCurrentIndex(4);
    ui->btnSend->setEnabled(false);

}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::on_btnOpen_clicked()
{
    if(ui->btnOpen->text() == "打开串口")
    {
        serial = new QSerialPort;
        //设置串口名
        serial->setPortName(ui->comboBox->currentText());
        //打开串口
        serial->open(QIODevice::ReadWrite);
        //设置波特率
        int bps = ui->comboBox_2->currentText().toInt();
        serial->setBaudRate(bps);
        //设置数据位数
        serial->setDataBits(QSerialPort::Data8);
        //设置奇偶校验
        serial->setParity(QSerialPort::NoParity);
        //设置停止位
        serial->setStopBits(QSerialPort::OneStop);
        //设置流控制
        serial->setFlowControl(QSerialPort::NoFlowControl);
        //关闭设置菜单使能
        ui->comboBox->setEnabled(false);
        ui->comboBox_2->setEnabled(false);
        ui->btnOpen->setText("关闭串口");
        ui->btnSend->setEnabled(true);
        //连接信号槽
        QObject::connect(serial, &QSerialPort::readyRead, this, &Dialog::Read_Data);
    }
    else
    {
        //关闭串口
        serial->clear();
        serial->close();
        serial->deleteLater();
        //恢复设置使能
        ui->comboBox->setEnabled(true);
        ui->comboBox_2->setEnabled(true);
        ui->btnOpen->setText("打开串口");
        ui->btnSend->setEnabled(false);
    }
}

void Dialog::on_btnSend_clicked()
{
    QString str = ui->textSend->text() + "\n";
    QByteArray buf = str.toLatin1();
    serial->write(buf);
}

void Dialog::Read_Data()
{
    buf = serial->readAll();

    if(!buf.isEmpty())
    {
        data+=QString(buf);
        int len=data.indexOf("\n");
        while(len>=0){
            Data=data.left(len);
            data=data.mid(len+1);
            len=data.indexOf("\n");
            ui->textRecv->append(Data);
            if(Data.startsWith("7MBD")){ //流水灯
                ui->textEdit_12->clear();ui->textEdit_12->append(Data.mid(4,1)[0]=='1'?"●":"○");
                ui->textEdit_13->clear();ui->textEdit_13->append(Data.mid(5,1)[0]=='1'?"●":"○");
                ui->textEdit_14->clear();ui->textEdit_14->append(Data.mid(6,1)[0]=='1'?"●":"○");
                ui->textEdit_15->clear();ui->textEdit_15->append(Data.mid(7,1)[0]=='1'?"●":"○");

                ui->textEdit_16->clear();ui->textEdit_16->append(Data.mid(8,1)[0]=='U'?"▅":"▂");
                ui->textEdit_17->clear();ui->textEdit_17->append(Data.mid(9,1)[0]=='U'?"▅":"▂");
                ui->textEdit_18->clear();ui->textEdit_18->append(Data.mid(10,1)[0]=='U'?"▅":"▂");
                ui->textEdit_19->clear();ui->textEdit_19->append(Data.mid(11,1)[0]=='U'?"▅":"▂");

            }
            else if(Data.startsWith("7MYD")){//原始数据
                 ui->textEdit_10->clear();ui->textEdit_10->append(Data.mid(4,4));
                 ui->textEdit->clear();ui->textEdit->append(Data.mid(8,6));
                 ui->textEdit_2->clear();ui->textEdit_2->append(Data.mid(14,6));
                 ui->textEdit_3->clear();ui->textEdit_3->append(Data.mid(20,6));
                 ui->textEdit_4->clear();ui->textEdit_4->append(Data.mid(26,6));
                 ui->textEdit_5->clear();ui->textEdit_5->append(Data.mid(32,6));
                 ui->textEdit_6->clear();ui->textEdit_6->append(Data.mid(38,6));
                }

            else if(Data.startsWith("7MRD")){
                 ui->textEdit_11->clear();ui->textEdit_11->append(Data.mid(4,4));
                 ui->textEdit_7->clear();ui->textEdit_7->append(Data.mid(8,6));
                 ui->textEdit_8->clear();ui->textEdit_8->append(Data.mid(14,6));
                 ui->textEdit_9->clear();ui->textEdit_9->append(Data.mid(20,6));
            }

        }

    }
    buf.clear();

}


void Dialog::on_btnClear_clicked()
{

    ui->textSend->clear();
}

void Dialog::on_pushButton_clicked()
{
    ui->textRecv->clear();
    ui->textEdit_12->clear();
    ui->textEdit_13->clear();
    ui->textEdit_14->clear();
    ui->textEdit_15->clear();
    ui->textEdit_16->clear();
    ui->textEdit_17->clear();
    ui->textEdit_18->clear();
    ui->textEdit_19->clear();
    ui->textEdit_10->clear();
    ui->textEdit->clear();
    ui->textEdit_2->clear();
    ui->textEdit_3->clear();
    ui->textEdit_4->clear();
    ui->textEdit_5->clear();
    ui->textEdit_6->clear();
    ui->textEdit_11->clear();
    ui->textEdit_7->clear();
    ui->textEdit_8->clear();
    ui->textEdit_9->clear();
}
