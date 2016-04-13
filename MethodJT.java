package fyp;

import java.awt.Color;
import java.awt.Graphics;

public class MethodJT extends Method {
	private static final long serialVersionUID = 1L;
    double stepScaleFactor=0.1; //set the m factor   	
	Matrix jacobian, jInverse, dX,dTheta,jJT,jJTDx,num,denom;
	double alpha = 0;

	public MethodJT(){  //setup the variables value	
	    screenPosX=840;
	    screenPosY=250;		
		maxIterations=100;// maximum iterations
		defaultSetting();  
	    dX= new Matrix(2,1);    
	    dTheta= new Matrix(numLinks,1);
	    jJT= new Matrix(2,2);
	    jJTDx= new Matrix(2,1);
	    num=new Matrix(1,1);
	    denom= new Matrix(1,1);

	    jacobian= new Matrix(2,numLinks);
	    jInverse= new Matrix(numLinks,2);
	            	    
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
    			 IK2DTopPanel.tfSuccess[4].setText("Yes");
    			stop();
    			break;
    		}// end if	
    		else{
    			 IK2DTopPanel.tfSuccess[4].setText("No");
    			 IK2DTopPanel.tfCount[4].setText(count+""); 
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
       dX= gl.subtractFrom(endPoint).multiply(stepScaleFactor);

       for(int i=0; i<numLinks; i++) {
       		jacobian.elements[0][i]= jacobian.elements[1][i]= 0;        
       		for(int j= i; j<numLinks; j++) {
       			jacobian.elements[0][i] += -lengths[j]*st[j];
       			jacobian.elements[1][i] += lengths[j]*ct[j];               
       		}           	
       }
       
       	jInverse= jacobian.pseudoInverse();     	
       	jJT= jacobian.multiply(jacobian.transpose());             
       	jJTDx= jJT.multiply(dX);
	   	num= dX.transpose().multiply(jJTDx);
	   	denom= jJTDx.transpose().multiply(jJTDx);       	    
	   	alpha=num.elements[0][0]/denom.elements[0][0];
	   	dTheta=jacobian.transpose().multiply(alpha).multiply(dX);
        thetas=ListOfEquation.CalTheta(numLinks, thetas, dTheta);
        System.out.printf("\n Count : %d" , count);
        System.out.printf("\n alpha : %f" , alpha);
    }  
    
    @Override
	public void paint(Graphics g){
		super.paint(g);   		
		draw2D.displayString(g,"Serif",12,"Jacobian Transpose",5, 180);
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