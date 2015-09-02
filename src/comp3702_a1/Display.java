package comp3702_a1;

import java.awt.Graphics;

import javax.swing.JFrame;
import javax.swing.JPanel;

public class Display extends JFrame {
	
	public Display() {
		setSize(400, 400);
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		setContentPane(new drawArea());
		setVisible(true);
	}
	
	public void draw() {
		setContentPane(new JPanel() {
			@Override
			protected void paintComponent(Graphics g) {
				g.drawLine(400, 400, 200, 200);
			}
		});
	}
}

class drawArea extends JPanel {
	@Override
	protected void paintComponent(Graphics g) {
		g.drawLine(100, 100, 200, 200);
	}
}