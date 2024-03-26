// float Setpoint, Input, Output;
// float M1Kp = 1, M1Ki = 0, M1Kd = 0;//激进方案
// float M2Kp = 0.8, M2Ki = 0.2, M2Kd = 0.5;//中等增益方案
// float M3Kp = 0.5, M3Ki = 0.1, M3Kd = 0.25;//保守方案
// float pout = 0, iout = 0, dout = 0;
// float Error=0,dt=0.1;//采样时间dt
// //数组建立

// float Dbuf[3];

// void setupPID(){
//     // Error[0] = 0;
//     // Dbuf[0] = 0;
// }
// void loop()
// {
//     Input = gps.course.deg()-courseTo;//每次的输入值
//     Error[2]=Error[1];
//     Error[1]=Error[0];
//     Error[0]=Input;
//     delay(1000*dt);
//     if (Input>60||Input<=180)//需要右转
//     {
//         Output=60;
//     }else if (Input<=60&&Input>30)
//     {
//         pout = M2Kd * Input;
//         iout += M2Ki * Input;
//         Output = pout + iout;
//    }else if (Input<=30&&Input>5)
//     {
//         pout = M2Kd * Input;
//         iout += M2Ki * Input;
//         Dbuf[2] = Dbuf[1];
//         Dbuf[1] = Dbuf[0];
//         Dbuf[0] = Error[0]-Error[1];
//         dout = M2Kd * Dbuf[0];
//         Output = pout + iout + dout;
//    }else if(Input<=5&&Input>=-5)
//    {
//     Output=0;
//    }else if(Input<-60||Input>=180)//需要左转
//     {
//         Output=60;
//     }else if (Input>=-60&&Input<-30)
//     {
//         pout = M2Kd * Input;
//         iout += M2Ki * Input;
//         Output = pout + iout;
//    }else if (Input>=-30&&Input<-5)
//     {
//         pout = M2Kd * Input;
//         iout += M2Ki * Input;
//         Dbuf[2] = Dbuf[1];
//         Dbuf[1] = Dbuf[0];
//         Dbuf[0] = Error[0]-Error[1];
//         dout = M2Kd * Dbuf[0];
//         Output = pout + iout + dout;
//    } else{Output=0;}
 //}
