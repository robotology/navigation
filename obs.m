  pkg load geometry 
  
  m_max_detection_distance = 5;
  m_min_detection_distance = 0.5;
  detection_distance = 5;
  m_robot_radius = 0.3;
  theta = 0;
  ctheta             = cos(theta-1.5707);
  stheta             = sin(theta-1.5707);
  detection_distance = m_min_detection_distance;

  
  m_enable_dynamic_max_distance=false;
%     if (m_enable_dynamic_max_distance)
%    {
%        %detection_distance is increased from min to max as the velocity of the robot increases
%        detection_distance = m_max_detection_distance * m_safety_coeff;
%    }

    %an obstacle farther than m_max_detection_distance is always ignored
    if (detection_distance>m_max_detection_distance)
        detection_distance = m_max_detection_distance;
endif

    %an obstacle nearer than m_min_detection_distance is always detected
    if (detection_distance<m_min_detection_distance)
        detection_distance = m_min_detection_distance;
        endif
        
  vertx(1) = (-m_robot_radius) * ctheta + detection_distance * (-stheta);
  verty(1) = (-m_robot_radius) * stheta + detection_distance * ctheta;
  vertx(2) = (+m_robot_radius) * ctheta + detection_distance * (-stheta);
  verty(2) = (+m_robot_radius) * stheta + detection_distance * ctheta;
  vertx(3) = +m_robot_radius  * ctheta;
  verty(3) = +m_robot_radius  * stheta;
  vertx(4) = -m_robot_radius  * ctheta;
  verty(4) = -m_robot_radius  * stheta;
  
  
  drawPolygon ([vertx(1) verty(1); vertx(2) verty(2);vertx(3) verty(3);vertx(4) verty(4);]);
hold on
grid on
drawCircle (0, 0, m_robot_radius);
drawCircle (0, 0, m_min_detection_distance);
drawCircle (0, 0, detection_distance);