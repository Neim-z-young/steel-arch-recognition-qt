#include "paramdialog.h"
#include "ui_paramdialog.h"

ParamDialog::ParamDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ParamDialog)
{
    ui->setupUi(this);
}

ParamDialog::~ParamDialog()
{
    delete ui;
}

void ParamDialog::on_buttonBox_accepted()
{
    if(ui->lineEditParam1->isModified() ||
            ui->lineEditParam2->isModified() ||
            ui->lineEditParam3->isModified() ||
            ui->lineEditParam4->isModified())
    {
        emit settingSignal(ui->lineEditParam1->text(),
                           ui->lineEditParam2->text(),
                           ui->lineEditParam3->text(),
                           ui->lineEditParam4->text());
        return;
    }
    emit cancelSignal();
}

void ParamDialog::on_buttonBox_rejected()
{
    emit cancelSignal();
}

void ParamDialog::closeEvent(QCloseEvent *event){
    emit cancelSignal();
}
