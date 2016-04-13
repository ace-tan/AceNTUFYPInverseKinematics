package fyp;

import java.awt.Color;
import java.awt.Graphics;
import javax.swing.JPanel;
import javax.swing.JTextField;

public class IK2DTopPanel extends JPanel {
    static final long serialVersionUID = 1L;
    static JTextField tfEEX = new JTextField(5);//mouse point
    static JTextField tfEEY = new JTextField(5);//mouse point
    static JTextField tfStepfactor = new JTextField(5);
    static JTextField[] tfL = new JTextField[3];
    static JTextField[] tfTheta = new JTextField[3];
    static JTextField[] tfSuccess = new JTextField [5]; 
    static JTextField[] tfCount= new JTextField[5];
    
    Color BackColor=new Color(238,238,238);// screen color
    int screenX=300;
    int screenY=250;
    int screenPosX=0;
    int screenPosY=0;
    int numLinks=3;
    int [] cartX= new int[4];// set x
    int [] cartY= new int[4];// set y
    double[] lengths = new double[numLinks];
    double[] thetas = new double[numLinks];
    double stepScaleFactor=10; //set the m factor
     
	public IK2DTopPanel(){
		
        setBackground (BackColor);
        setSize(screenX, screenY);
        setLayout(null);
        this.setLocation(screenPosX,screenPosY);
        this.setVisible(true);
 
        for(int i=0;i<=numLinks; i++){
        	cartX[i]=0+20*i;
        	cartY[i]=0;
        }
        
        for(int i=0; i<5 ; i++){
        	tfSuccess[i]= new JTextField(5);
      		tfCount[i]= new JTextField(5);
      		tfCount[i].setLocation(130, 60+20*i);
            tfCount[i].setSize(35, 20);
            tfCount[i].setBackground(Color.white);
      	  	tfCount[i].setEditable(false);    
      	  	tfCount[i].setText("");        	
      	  	this.add(tfCount[i]);
      		tfSuccess[i]= new JTextField(5);
      		tfSuccess[i].setLocation(240, 60+20*i);
            tfSuccess[i].setSize(25, 20);
            tfSuccess[i].setBackground(Color.white);
      	  	tfSuccess[i].setEditable(false);    
      	  	tfSuccess[i].setText("");        	
      	  	this.add(tfSuccess[i]);
        }
        
    	for(int i=0; i<numLinks; i++) {    
            lengths[i]=20;
            thetas[i]=0;
            tfL[i]= new JTextField(5);
            tfL[i].setLocation(22+50*i, 222);
            tfL[i].setSize(25, 20);
            tfL[i].setBackground(Color.white);
            tfL[i].setEditable(false);
            tfL[i].setText(String.format("%.0f",lengths[i]));//cast to string
            this.add(tfL[i]);
               
            tfTheta[i]= new JTextField(5);
            tfTheta[i].setLocation(172+50*i, 222);
            tfTheta[i].setSize(25, 20);
            tfTheta[i].setBackground(Color.white);
            tfTheta[i].setEditable(false);
            tfTheta[i].setText(String.format("%.0f",thetas[i]));//cast to string
            this.add(tfTheta[i]);
    	} 
    	
        tfEEX.setLocation(32, 180);
        tfEEX.setSize(30, 20);
        tfEEX.setBackground(Color.white);
        tfEEX.setEditable(false);
        tfEEX.setText("");
        this.add(tfEEX);
        
        tfEEY.setLocation(90, 180);
        tfEEY.setSize(30, 20);
        tfEEY.setBackground(Color.white);
        tfEEY.setEditable(false);
        tfEEY.setText("");
        this.add(tfEEY);
        
        tfStepfactor.setLocation(165, 180);
        tfStepfactor.setSize(25, 20);
        tfStepfactor.setBackground(Color.white);
        tfStepfactor.setEditable(false);
        tfStepfactor.setText("10");//cast to string   
        this.add(tfStepfactor); 
	}
	
	public void paint(Graphics g){
		super.paint(g);     

        draw2D.displayString(g,"Serif",17,"Inverse kinematics 2D",5,18);
        draw2D.displayString(g,"Serif",12,"Author : Tan Caibao (U1121615B)",5, 35);
        draw2D.displayString(g,"Serif",17,"Methods:              Counts:              Success:",5, 58);
        draw2D.displayString(g,"Serif",12,"PINV:",5, 75);
        draw2D.displayString(g,"Serif",12,"CCD:",5, 95);
        draw2D.displayString(g,"Serif",12,"Secondary:",5, 115);
        draw2D.displayString(g,"Serif",12,"DLS:",5, 135);
        draw2D.displayString(g,"Serif",12,"Jacobian Tranpose:",5, 155);   	
        draw2D.displayString(g,"Serif",17,"End Effector:              Stepfactor:",5, 175);
        draw2D.displayString(g,"Serif",12,"EEX:           EEY:                     m:",5, 195);
        draw2D.displayString(g,"Serif",17,"Lengths:                                  Thetas:",5, 215);
        draw2D.displayString(g,"Serif",12,"L1:           L2:           L3:           T1:           T2:           T3:           ",5, 235);
	}
}//outer class