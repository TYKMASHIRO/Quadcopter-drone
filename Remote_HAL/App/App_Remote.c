#include "App_Remote.h"

struct _Rc rc;
struct _Offset offset;
struct _Filter Filter_Thr, Filter_Pitch, Filter_Roll, Filter_Yaw;

extern uint16_t ADC_Value[4];

/**
 * @description: ȡ���N�ε�ƽ��ֵ
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Window_Filter(struct _Rc *rc)
{
    static uint16_t count = 0;
    /* 1������������ */
    Filter_Thr.sum -= Filter_Thr.old[count];
    Filter_Pitch.sum -= Filter_Pitch.old[count];
    Filter_Roll.sum -= Filter_Roll.old[count];
    Filter_Yaw.sum -= Filter_Yaw.old[count];

    /* 2������������:������Ҫ�ӵ�sum�ͬʱҲҪ�浽old�� */
    Filter_Thr.old[count] = rc->THR;
    Filter_Pitch.old[count] = rc->PIT;
    Filter_Roll.old[count] = rc->ROL;
    Filter_Yaw.old[count] = rc->YAW;

    Filter_Thr.sum += Filter_Thr.old[count];
    Filter_Pitch.sum += Filter_Pitch.old[count];
    Filter_Roll.sum += Filter_Roll.old[count];
    Filter_Yaw.sum += Filter_Yaw.old[count];

    /* 3����ƽ��ֵ */
    rc->THR = Filter_Thr.sum / Filter_Num;
    rc->PIT = Filter_Pitch.sum / Filter_Num;
    rc->ROL = Filter_Roll.sum / Filter_Num;
    rc->YAW = Filter_Yaw.sum / Filter_Num;

    /* 4���ж�����ֵ�ķ�Χ */
    count++;
    if (count >= 10)
    {
        count = 0;
    }
}

/**
 * @description: ��������������ҡ���е�У׼���������У׼��
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Mid_Offset(struct _Rc *rc)
{
    static uint16_t key_count = 0;
    static uint32_t sum_thr = 0, sum_pit = 0, sum_roll = 0, sum_yaw = 0;
    static uint16_t count = 0;
    /* �жϰ������� */
    if (!READ_KEY_LEFT_X)
    {
        printf("start offset...\r\n");
        key_count++;
        if (key_count >= 20) // ��ʱ����main����while�е���ʱ������
        {
            /* �����ˣ���ʼУ׼�е� =�� ƫ��ֵ = 50�εĺ�/50 - 1500 */
            if (count == 0)
            {
                /* count=0ʱ�����г�ʼ���������ۼ�ֵ��ƫ��ֵ */
                sum_thr = 0;
                sum_pit = 0;
                sum_roll = 0;
                sum_yaw = 0;
                offset.THR = 0;
                offset.PIT = 0;
                offset.ROL = 0;
                offset.YAW = 0;
                /* count+1���´β��ٽ���ʼ�� */
                count = 1;
                return;
            }
            else
            {
                /* ��ʼ�ۼ�50�Ρ�������2-51�� */
                count++;
                sum_thr += rc->THR;
                sum_pit += rc->PIT;
                sum_roll += rc->ROL;
                sum_yaw += rc->YAW;
            }
            if (count == 51)
            {
                count--;                             // ��������count�Ѿ���51��ʵ���ۼ���50��
                offset.THR = sum_thr / count - 1000; // �������⣬���У׼
                offset.PIT = sum_pit / count - 1500;
                offset.ROL = sum_roll / count - 1500;
                offset.YAW = sum_yaw / count - 1500;

                /* ��ȡ��ƫ��ֵ����ȥ����count��key_count */
                count = 0;
                key_count = 0;
            }
        }
    }
}

/**
 * @description: ���ֵ����Сֵ�޷�
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Limit(struct _Rc *rc)
{
    rc->THR = (rc->THR < 1000) ? 1000 : rc->THR;
    rc->THR = (rc->THR > 2000) ? 2000 : rc->THR;

    rc->PIT = (rc->PIT < 1000) ? 1000 : rc->PIT;
    rc->PIT = (rc->PIT > 2000) ? 2000 : rc->PIT;

    rc->ROL = (rc->ROL < 1000) ? 1000 : rc->ROL;
    rc->ROL = (rc->ROL > 2000) ? 2000 : rc->ROL;

    rc->YAW = (rc->YAW < 1000) ? 1000 : rc->YAW;
    rc->YAW = (rc->YAW > 2000) ? 2000 : rc->YAW;
}

/**
 * @description: �е��޷�����Ϊ���м丽��������Ҫ��ֵ
 * @param {_Rc} *rc
 * @return {*}
 */
void App_Remote_Mid_Limit(struct _Rc *rc)
{
    /* ���ǵ����Ų���ص����ֶ�������ȫ���м���ѣ���Χ����һ�� */
    if (rc->THR > 1450 && rc->THR < 1550)
    {
        rc->THR = 1500;
    }

    /* ��������ҡ�˻��Զ����У���Χ���Ը�Сһ�� */
    if (rc->PIT > 1490 && rc->PIT < 1510)
    {
        rc->PIT = 1500;
    }
    if (rc->ROL > 1490 && rc->ROL < 1510)
    {
        rc->ROL = 1500;
    }
    if (rc->YAW > 1490 && rc->YAW < 1510)
    {
        rc->YAW = 1500;
    }
}

// /**
//  * @description: ҡ��ɨ��
//  * @return {*}
//  */
void App_Remote_Stick_Scan()
{
    /* 1����ADC��ֵת��Ϊ 1000-2000�ķ�Χ�����Ҵ����� */
    /*
        0: ��ҡ�˵�����
        1: ��ҡ�˵�����
        2: ��ҡ�˵�����
        3: ��ҡ�˵�����
     */
    rc.THR = 2000 - (uint16_t)(0.25f * ADC_Value[1]) - offset.THR;
    rc.PIT = 2000 - (uint16_t)(0.25f * ADC_Value[3]) - offset.PIT;
    rc.ROL = 2000 - (uint16_t)(0.25f * ADC_Value[2]) - offset.ROL;
    rc.YAW = 2000 - (uint16_t)(0.25f * ADC_Value[0]) - offset.YAW;

    /* ���������˲����仯���ƽ�� */
    App_Remote_Window_Filter(&rc);
    /* �е�У׼���������У׼�������е�У׼ */
    // App_Remote_Mid_Offset(&rc);
    /* �����С�޷� */
    App_Remote_Limit(&rc);
    /* �е��޷� */
    App_Remote_Mid_Limit(&rc);
}

/**
 * @description: �����ĸ�����΢��������ǰ�������ˣ�����������ơ����ƣ�
 * @return {*}
 */
void App_Remote_KeyPress()
{
    /*
        ������־λ����=
        if(��������&& ��־λ==0)
        {
            �ӳ�һ��
            if(�������� && ��־λ==0)
            {
                �����߼���
                ��־=1��
            }

        }
        if(�����ɿ� && ��־λ=1)
        {
            ��־λ=0��
        }
     */

    /* 1���ж��ĸ��������£���Ӧ��ȥ΢�� */
    if (!READ_KEY_U)
    {
        /* ǰ���������£�΢��pitch */
        offset.PIT = offset.PIT - 10; // ǰ���ü�����pitch���=������ƫ��ֵ��С��
    }
    /* Ϊʲô����else if:�û��п��ܶ������ͬʱ�������ж���Ч�Ļ�������if */
    if (!READ_KEY_D)
    {
        /* ǰ���������£�΢��pitch */
        offset.PIT = offset.PIT + 10; // �����üӣ���pitch��С=������ƫ��ֵ���
    }
    if (!READ_KEY_L)
    {
        /* ��߰������£�΢��roll */
        offset.ROL = offset.ROL + 10; // �����üӣ���roll��С=������ƫ��ֵ���
    }
    if (!READ_KEY_R)
    {
        /* ��߰������£�΢��roll */
        offset.ROL = offset.ROL - 10; // �����ü�����roll���=������ƫ��ֵ��С��
    }
}

/**
 * @description: ����ͨ��Э�飬��װҪ���͸��ɿص�����
 * @return {*}
 */
void App_Remote_RemoteData(uint8_t *remote_send)
{
    uint8_t index = 0;
    uint32_t check_sum = 0;
    /* Э�飺֡ͷ+������+����+����+У��� */
    /* 1 ֡ͷ */
    remote_send[index++] = 0xAA;
    remote_send[index++] = 0xAF;
    /* 2 ������ */
    remote_send[index++] = 0x03;
    /* 3 ���� */
    remote_send[index++] = 0x0; // �ȸ���0��������������ȥ
    /* 4 ң������ */
    /* 4.1 ҡ������: THR -�� YAW -�� ROL -�� PIT��ÿ������Ϊ ��8λ����8λ */
    remote_send[index++] = (uint8_t)(rc.THR >> 8);
    remote_send[index++] = (uint8_t)rc.THR;
    remote_send[index++] = (uint8_t)(rc.YAW >> 8);
    remote_send[index++] = (uint8_t)rc.YAW;
    remote_send[index++] = (uint8_t)(rc.ROL >> 8);
    remote_send[index++] = (uint8_t)rc.ROL;
    remote_send[index++] = (uint8_t)(rc.PIT >> 8);
    remote_send[index++] = (uint8_t)rc.PIT;
    /* 4.2 ����ͨ��aux1-aux6������û�ã����Բ�д��Ҳ����д */
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;
    remote_send[index++] = 0x0;

    /* 5 ���¼������ݳ��� */
    remote_send[3] = index - 4;

    /* 6 У��� */
    for (uint8_t i = 0; i < index; i++)
    {
        check_sum += remote_send[i];
    }
    remote_send[index++] = (uint8_t)(check_sum >> 24);
    remote_send[index++] = (uint8_t)(check_sum >> 16);
    remote_send[index++] = (uint8_t)(check_sum >> 8);
    remote_send[index++] = (uint8_t)(check_sum);
}
