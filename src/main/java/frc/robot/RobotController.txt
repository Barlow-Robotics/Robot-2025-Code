package frc.robot;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;

public class RobotController extends JPanel {

    // Store the points clicked
    private java.util.List<Point> points = new ArrayList<>();
    private RobotCommunicator communicator;

    public RobotController(RobotCommunicator communicator) {
        this.communicator = communicator;
        setPreferredSize(new Dimension(400, 400));

        addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                if (points.size() < 12) {
                    points.add(e.getPoint());
                    repaint();
                    sendCoordinatesToRobot(e.getPoint());
                } else {
                    JOptionPane.showMessageDialog(null, "12 points have been clicked.");
                }
            }
        });
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        for (Point point : points) {
            g.fillOval(point.x - 5, point.y - 5, 10, 10);
        }
    }

    private void sendCoordinatesToRobot(Point point) {
        communicator.sendCoordinates(point);
    }
}
