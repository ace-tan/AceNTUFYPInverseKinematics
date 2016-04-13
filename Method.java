package fyp;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.geom.Ellipse2D;

public class Method  extends Canvas implements MouseListener, Runnable{
	private static final long serialVersionUID = 1L;
	//default
    boolean begin,mouseClicked;
    Color BackColor=new Color(238,238,238);// screen color
    int cursorX=0; //mouse cursor x
    int cursorY=0; //mouse cursor y
	int goalMSetX=0;//cartesian goal point 
	int goalMSetY=0;//cartesian goal point	
	int count=0;//begin iterations	
	int screen=200;
    int screenX=200;
    int screenY=200;
    int screenPosX;
    int screenPosY;	
    int numLinks=3;    
    int maxIterations;// maximum iterations		
    
	Integer [] x =new Integer[4];//original java paint 
	Integer [] y =new Integer[4];//original java paint 	
	double [] lengths = new double[numLinks];
    Double [] ct= new Double[numLinks];
    Double [] st= new Double[numLinks];
    Double [] thetas = new Double[numLinks]; //default radians		
    Matrix [] jointLocations = new Matrix[numLinks]; 
	Matrix goal, endPoint;	
	static Thread thread;
		
	public void defaultSetting(){
		begin = true; 		
	    mouseClicked=false;
	    goal = new Matrix(2,1);
	    endPoint= new Matrix(2,1);
	    
	    for(int i=0; i<numLinks; i++) {
	        jointLocations[i]= new Matrix(2,1);
	        jointLocations[i].elements[0][0]= 20+20*i;
	        jointLocations[i].elements[1][0]= 0;
	        thetas[i]= 0.0;
	        lengths[i]= 20;  
	    }
	}
	
	public void mouseClicked(MouseEvent e) {}
	public void mouseEntered(MouseEvent arg0) {}
	public void mouseExited(MouseEvent arg0) {}
	public void mouseReleased(MouseEvent arg0) {}
	public void mousePressed(MouseEvent e) {
		Rectangle bounds = bounds();
		mouseClicked=true;    				
		cursorX = e.getX();
		cursorY = e.getY();    		
		goalMSetX= e.getX()-bounds.width/2;
		goalMSetY= (bounds.height/2-e.getY());  		
        IK2DTopPanel.tfEEX.setText(goalMSetX+"");//cast to string
        IK2DTopPanel.tfEEY.setText(goalMSetY+"");//cast to string
//		System.out.printf("\ncx: %d",goalMSetX);	
//		System.out.printf(" cy: %d",goalMSetY);	
		count=0;//set the count to 0    
		Matrix.setGoal(goal,goalMSetX, goalMSetY);
		start();        
	}
          
	public void paint(Graphics g){
		super.paint(g);
		defaultPaint(g);
    	Graphics2D g2 = (Graphics2D)g;
        Ellipse2D.Double circle2 = new Ellipse2D.Double(40, 40, 120, 120);
        g2.setPaint(Color.black);
        g2.draw(circle2);
	}
     
	public void defaultPaint(Graphics g){
        draw2D.originLine(g,screen);        
	}
	 
	public void stop() {
        if (thread != null)
	          thread.interrupt();
	    thread = null;
	}		
	
 	public void start() {
        thread = new Thread(this);
        thread.setPriority(Thread.MIN_PRIORITY);
        thread.start();
   }

    public void run() {
        Thread me = Thread.currentThread();
        while (thread == me) {
        	try {
					Thread.sleep(1000/5);
				} catch (InterruptedException e) {}
        	repaint();
        }
        thread = null;
   } 
}//inner class