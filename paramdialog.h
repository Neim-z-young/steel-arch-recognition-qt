#ifndef PARAMDIALOG_H
#define PARAMDIALOG_H

#include <QDialog>
#include <QCloseEvent>

namespace Ui {
class ParamDialog;
}

class ParamDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ParamDialog(QWidget *parent = nullptr);
    ~ParamDialog();

signals:
    void cancelSignal();
    void settingSignal(QString p1, QString p2, QString p3, QString p4);

protected:
    void closeEvent(QCloseEvent *event);    // 重写closeEvent

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:
    Ui::ParamDialog *ui;
};

#endif // PARAMDIALOG_H
