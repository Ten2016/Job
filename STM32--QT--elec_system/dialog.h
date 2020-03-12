#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

private slots:

    void on_btnOpen_clicked();

    void on_btnSend_clicked();

    void Read_Data();

    void on_btnClear_clicked();

    void on_pushButton_clicked();

private:
    Ui::Dialog *ui;
    QSerialPort *serial;

    QString Data;
    QByteArray buf;
    QString data;
};

#endif // DIALOG_H
