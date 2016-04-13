package fyp;

import java.awt.Color;
import java.awt.Graphics;

public class MethodPINV  extends Method {
	private static final long serialVersionUID = 1L;
	double stepScaleFactor=0.1; //set the m=10 
    Matrix jacobian, jInverse, dX,dTheta;
           
	public MethodPINV(){  //setup the variables value	
		maxIterations=100;// maximum iterations		
		screenPosX=0;
        screenPosY=250; 
        defaultSetting();    
	    dTheta= new Matrix(numLinks,1);
	    jacobian= new Matrix(2,numLinks);
	    jInverse= new Matrix(numLinks,2);
	    dX= new Matrix(2,1);
        this.setBackground (Color.white);
        this.setSize(screenX,screenY);
        this.setLocation(screenPosX,screenPosY);
        this.setVisible(true);
        this.addMouseListener(this);
	}		
   	
	 public void condition() {
     	while (count<=maxIterations) {
     		if( (Math.sqrt(Math.abs(endPoint.subtractFrom(goal).elements[0][0]))*2 
         			+(Math.abs(endPoint.subtractFrom(goal).elements[1][0]))*2) <=1) {
     			IK2DTopPanel.tfSuccess[0].setText("Yes");
     			stop();
     			break;
     		}// end if	
     		else{
     			IK2DTopPanel.tfSuccess[0].setText("No");
     			IK2DTopPanel.tfCount[0].setText(count+"");        			
       			// make one iteration towards to goal and repaint the arm
       			moveToGoal_NLink(goal);
       			if(count==maxIterations)
       				stop();
         	}
         }//end while
     }//end condition
            
     public void moveToGoal_NLink(Matrix gl) {
     	++count;//increment
         ct=ListOfEquation.Calct(ct,numLinks,thetas);
         st=ListOfEquation.Calst(st,numLinks,thetas);

         for(int i=0; i<numLinks; i++) {    
         	if(i==0){
         		 jointLocations[0].elements[0][0]= lengths[0]*ct[0];
                  jointLocations[0].elements[1][0]= lengths[0]*st[0];          
         	}
         	else{
         		jointLocations[i].elements[0][0]= jointLocations[i-1].elements[0][0] + lengths[i]*ct[i];
         		jointLocations[i].elements[1][0]= jointLocations[i-1].elements[1][0] + lengths[i]*st[i];
         	}
         }
	    
         endPoint= jointLocations[numLinks-1];// last point
         // dX is a vector in the direction of the end-effector to the goal point
         dX= gl.subtractFrom(endPoint).multiply(stepScaleFactor);
         
         for(int i=0; i<numLinks; i++) {
         	jacobian.elements[0][i]= jacobian.elements[1][i]= 0;
        
         	for(int j= i; j<numLinks; j++) {
         		jacobian.elements[0][i] += -lengths[j]*st[j];
         		jacobian.elements[1][i] += lengths[j]*ct[j];            
         	}     	
         }
         
         jInverse= jacobian.pseudoInverse(); 
         dTheta= jInverse.multiply(dX);           
         thetas=ListOfEquation.CalTheta(numLinks, thetas, dTheta);
         
         System.out.printf("\n Count : %d" , count);
         System.out.printf("\n Joint Node 0 : %f , %f  " , jointLocations[0].elements[0][0], jointLocations[0].elements[1][0]);
		 System.out.printf("\n Joint Node 1 : %f , %f  " , jointLocations[1].elements[0][0], jointLocations[0].elements[1][0]);
		 System.out.printf("\n End Effector : %f , %f  " , jointLocations[2].elements[0][0], jointLocations[0].elements[1][0]);
		 System.out.printf("\n Dx : %f , %f  " , dX.elements[0][0], dX.elements[1][0]);
		 System.out.printf("\n Jacobian row 0 : %f , %f , %f" , jacobian.elements[0][0], jacobian.elements[0][1], jacobian.elements[0][2]);
		 System.out.printf("\n Jacobian row 1 : %f , %f , %f " , jacobian.elements[1][0], jacobian.elements[1][1], jacobian.elements[1][2]);		    
		 System.out.printf("\n JInverse row 0 : %f , %f  " , jInverse.elements[0][0], jInverse.elements[0][1]);
		 System.out.printf("\n JInverse row 1 : %f , %f  " , jInverse.elements[1][0], jInverse.elements[1][1]);
		 System.out.printf("\n JInverse row 2 : %f , %f  " , jInverse.elements[2][0], jInverse.elements[2][1]);
		 System.out.printf("\n Theta : %f , %f , %f  " , ((thetas[0]*180/Math.PI)%180), ((thetas[1]*180/Math.PI)%180), ((thetas[2]*180/Math.PI)%180));
		 System.out.println("");		    
     }         
        
     @Override
 	public void paint(Graphics g){
 		super.paint(g);		
 		draw2D.displayString(g,"Serif",12,"PINV",5, 180);
 		x[0] =screen/2;	//default center point
	    y[0] =screen/2; //default center point
	    Color [] newColor= new Color[10];
	    newColor[0]=Color.yellow;
        newColor[1]=Color.blue;
	    newColor[2]=Color.GRAY;
	    newColor[3]=Color.green;	        
 		
	    if(mouseClicked==true){		
 			draw2D.drawCircle(g,cursorX,cursorY,Color.red);//draw the target
 			condition();
 		}	  
 		
 	    if(begin==true){
 			for(int i=0; i<numLinks; i++) {
     	        x[i+1] = (int)jointLocations[i].elements[0][0]+screen/2;
     	        y[i+1] = screen/2-(int)jointLocations[i].elements[1][0];
     	    } 			
 		}
 	       		
 		for(int i=0; i<numLinks;i++){           				 //convert radians to degree
 			IK2DTopPanel.tfTheta[i].setText(String.format("%.0f",(ListOfEquation.convertRadtoDegree(thetas[i]))%180));//cast to string
 		}
 		
         draw2D.displayLine(g,x,y);
         draw2D.displayNode(g,x,y,newColor,numLinks);
 	} 	      
}