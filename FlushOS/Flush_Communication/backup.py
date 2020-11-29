if ((state == 0)) {
        if (data == 0xBD) {
            state++;
        } else {
            state = 0;
            result = 0xFA;
        }
    } else if (state == 1) {
        if (data == ID) {
            checksum += data;
            state++;
        } else {
            state = 0;
            result = 0xFA;
        }
    } else if (state == 2) {
        checksum += data;
        pose_f_UART = data << 8;
        state++;
    } else if (state == 3) {
        checksum += data;
        pose_f_UART |= data;
        pose_f_UART = pose_f_UART; // 154
        state++;
    } else if (state == 4) {
        checksum += data;
        t_UART = data << 8;
        state++;
    } else if (state == 5) {
        checksum += data;
        t_UART |= data;
        state++;
    } else if (state == 6) {
        checksum += data;
        theta_UART = data << 8;
        state++;
    } else if (state == 7) {
        checksum += data;
        theta_UART |= data;
        state++;
    } else if (state == 8) {
        kchecksum = ~checksum;
        checksum = checsum & 0xFF;
        if (checksum == data) {
            theta = theta_UART;
            theta = theta - 180;
            t = t_UART;
            t = t / 1000.0;
            pose_f = pose_f_UART;
            result = 0xAC;
            printf("%.2f\t", theta);
            printf("%.2f\t", pose_f);
            printf("%.2f\n", t);
        } else {
            result = 0xFA;
        }
        theta_UART = 0;
        t_UART = 0;
        pose_f_UART = 0;
        checksum = 0;
        state = 0;
        En_T1 = 40;

    }
    return result;