import React, { useState, useEffect, useRef, useCallback } from 'react';

// Type definitions
interface Point {
  x: number;
  y: number;
}

interface VehicleState {
  x: number;
  y: number;
  angle: number;
  velocity: number;
  angularVelocity: number;
}

interface TrailerState extends VehicleState {
  hitchLength: number;
  wheelbase: number;
}

interface VelocityVector {
  x: number;
  y: number;
}

interface AckermannAngles {
  leftAngle: number;
  rightAngle: number;
}

interface SimulationParams {
  dt: number;
  velocity: number;
  steeringAngle: number;
  wheelbase: number;
  trackWidth: number;
  trailerCount: number;
  hitchLength: number;
  trailerWheelbase: number;
}

// 1. Basic Kinematics Model Corrections
class VehicleKinematics {
  /**
   * Calculate velocity vector from scalar velocity and heading angle
   * 修正後: 速度ベクトル = [V * cos(θ), V * sin(θ)]
   */
  static getVelocityVector(velocity: number, angle: number): VelocityVector {
    return {
      x: velocity * Math.cos(angle),
      y: velocity * Math.sin(angle)
    };
  }

  /**
   * Calculate scalar velocity from velocity vector
   */
  static getScalarVelocity(velocityVector: VelocityVector): number {
    return Math.sqrt(velocityVector.x ** 2 + velocityVector.y ** 2);
  }
}

// 2. Trailer Connection Kinematics Corrections
class TrailerKinematics {
  /**
   * Calculate hitch point velocity with corrected vector operations
   * 修正後: ヒッチ点速度ベクトル = vec_vi-1 + ωi-1 × vec_ri-1
   */
  static calculateHitchVelocity(
    prevVelocity: VelocityVector,
    prevAngularVelocity: number,
    hitchLength: number,
    prevAngle: number
  ): VelocityVector {
    return {
      x: prevVelocity.x - prevAngularVelocity * hitchLength * Math.sin(prevAngle),
      y: prevVelocity.y + prevAngularVelocity * hitchLength * Math.cos(prevAngle)
    };
  }

  /**
   * Calculate trailer angular velocity with proper coordinate transformation
   * 修正後: 正しい座標変換を使用
   */
  static calculateTrailerAngularVelocity(
    hitchVel: VelocityVector,
    trailerAngle: number,
    wheelbase: number
  ): number {
    // ヒッチ点速度をトレーラー座標系に変換
    const vx_local = hitchVel.x * Math.cos(trailerAngle) + hitchVel.y * Math.sin(trailerAngle);
    const vy_local = -hitchVel.x * Math.sin(trailerAngle) + hitchVel.y * Math.cos(trailerAngle);
    
    // 角速度は横方向速度成分から計算
    return vy_local / wheelbase;
  }
}

// 3. Ackermann Geometry Corrections
class AckermannGeometry {
  /**
   * Calculate Ackermann steering angles with proper inner/outer wheel handling
   * 修正後: 内輪と外輪の角度計算を正確に実装
   */
  static calculateAckermannAngles(
    steeringAngle: number,
    wheelbase: number,
    trackWidth: number
  ): AckermannAngles {
    const steeringRad = steeringAngle * Math.PI / 180;
    
    if (Math.abs(steeringRad) < 0.001) {
      return { leftAngle: 0, rightAngle: 0 };
    }
    
    const R = wheelbase / Math.tan(Math.abs(steeringRad));
    const sign = steeringRad > 0 ? 1 : -1;
    
    // 内輪と外輪の角度計算を修正
    const innerR = R - sign * trackWidth / 2;
    const outerR = R + sign * trackWidth / 2;
    
    const innerAngle = sign * Math.atan(wheelbase / innerR) * 180 / Math.PI;
    const outerAngle = sign * Math.atan(wheelbase / outerR) * 180 / Math.PI;
    
    return steeringRad > 0 ? 
      { leftAngle: innerAngle, rightAngle: outerAngle } :
      { leftAngle: outerAngle, rightAngle: innerAngle };
  }
}

// 4. Stanley Controller Improvements
class StanleyController {
  /**
   * Calculate steering angle using improved Stanley control law
   * 修正後: 安全な最小速度設定と適切な符号処理
   */
  static calculateSteeringAngle(
    vehicleState: VehicleState,
    pathPoints: Point[],
    k: number = 3.5
  ): number {
    const { nearestPoint, crossTrackError } = this.findNearestPointWithError(vehicleState, pathPoints);
    const headingError = this.calculateHeadingError(vehicleState, nearestPoint, pathPoints);
    
    // 安全のための最小速度設定を明確化
    const minVelocity = 0.1; // [m/s] ゼロ除算防止
    const effectiveVelocity = Math.max(Math.abs(vehicleState.velocity), minVelocity);
    
    // Stanley制御則（符号を適切に処理）
    const crossTrackTerm = Math.atan2(k * crossTrackError, effectiveVelocity);
    const steeringAngle = headingError + crossTrackTerm;
    
    // 操舵角制限
    const maxSteeringAngle = 35; // [degrees]
    return Math.max(-maxSteeringAngle, Math.min(maxSteeringAngle, steeringAngle * 180 / Math.PI));
  }

  private static findNearestPointWithError(
    vehicleState: VehicleState,
    pathPoints: Point[]
  ): { nearestPoint: Point; crossTrackError: number } {
    let minDistance = Infinity;
    let nearestPoint = pathPoints[0];
    let crossTrackError = 0;

    for (const point of pathPoints) {
      const distance = Math.sqrt(
        (point.x - vehicleState.x) ** 2 + (point.y - vehicleState.y) ** 2
      );
      if (distance < minDistance) {
        minDistance = distance;
        nearestPoint = point;
        // Calculate cross track error (simplified)
        crossTrackError = distance * Math.sign(
          (point.x - vehicleState.x) * Math.sin(vehicleState.angle) -
          (point.y - vehicleState.y) * Math.cos(vehicleState.angle)
        );
      }
    }

    return { nearestPoint, crossTrackError };
  }

  private static calculateHeadingError(
    vehicleState: VehicleState,
    nearestPoint: Point,
    pathPoints: Point[]
  ): number {
    // Simplified heading error calculation
    const targetAngle = Math.atan2(
      nearestPoint.y - vehicleState.y,
      nearestPoint.x - vehicleState.x
    );
    let headingError = targetAngle - vehicleState.angle;
    
    // Normalize angle to [-π, π]
    while (headingError > Math.PI) headingError -= 2 * Math.PI;
    while (headingError < -Math.PI) headingError += 2 * Math.PI;
    
    return headingError;
  }
}

// 5. Jackknife Prevention Improvements
class JackknifeController {
  /**
   * Prevent jackknife by constraining relative angles between vehicles
   * 修正後: 角度正規化と適切な制約処理
   */
  static preventJackknife(
    angles: number[],
    maxRelativeAngle: number = 85 * Math.PI / 180
  ): number[] {
    const constrainedAngles = [...angles];
    
    for (let i = 1; i < constrainedAngles.length; i++) {
      let relativeAngle = constrainedAngles[i] - constrainedAngles[i - 1];
      
      // 角度を[-π, π]に正規化
      while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
      while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;
      
      if (Math.abs(relativeAngle) > maxRelativeAngle) {
        const sign = relativeAngle > 0 ? 1 : -1;
        constrainedAngles[i] = constrainedAngles[i - 1] + sign * maxRelativeAngle;
      }
    }
    
    return constrainedAngles;
  }
}

// 6. Numerical Stability Improvements
class NumericalIntegrator {
  /**
   * Update vehicle state with improved numerical stability
   * 修正後: 中点法とより安定した積分
   */
  static updateVehicleState(
    state: VehicleState,
    velocity: number,
    steeringAngle: number,
    dt: number,
    wheelbase: number
  ): VehicleState {
    const steeringRad = steeringAngle * Math.PI / 180;
    
    // 極小角度での数値不安定性を回避
    let angularVelocity: number;
    if (Math.abs(steeringRad) < 1e-6) {
      angularVelocity = 0;
    } else {
      angularVelocity = velocity * Math.tan(steeringRad) / wheelbase;
    }
    
    // 中点法による積分
    const newAngle = state.angle + angularVelocity * dt;
    const avgAngle = (state.angle + newAngle) / 2;
    
    return {
      x: state.x + velocity * Math.cos(avgAngle) * dt,
      y: state.y + velocity * Math.sin(avgAngle) * dt,
      angle: newAngle,
      velocity: velocity,
      angularVelocity: angularVelocity
    };
  }

  /**
   * Update trailer state using corrected kinematics
   */
  static updateTrailerState(
    trailerState: TrailerState,
    prevState: VehicleState,
    dt: number
  ): TrailerState {
    const prevVelocityVector = VehicleKinematics.getVelocityVector(
      prevState.velocity,
      prevState.angle
    );

    const hitchVelocity = TrailerKinematics.calculateHitchVelocity(
      prevVelocityVector,
      prevState.angularVelocity,
      trailerState.hitchLength,
      prevState.angle
    );

    const angularVelocity = TrailerKinematics.calculateTrailerAngularVelocity(
      hitchVelocity,
      trailerState.angle,
      trailerState.wheelbase
    );

    const velocity = VehicleKinematics.getScalarVelocity(hitchVelocity);
    
    // 中点法による積分
    const newAngle = trailerState.angle + angularVelocity * dt;
    const avgAngle = (trailerState.angle + newAngle) / 2;

    return {
      ...trailerState,
      x: trailerState.x + velocity * Math.cos(avgAngle) * dt,
      y: trailerState.y + velocity * Math.sin(avgAngle) * dt,
      angle: newAngle,
      velocity: velocity,
      angularVelocity: angularVelocity
    };
  }
}

// Main Simulator Component
const MultiTrailerSimulator: React.FC = () => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationRef = useRef<number>();
  
  const [isRunning, setIsRunning] = useState(false);
  const [vehicleState, setVehicleState] = useState<VehicleState>({
    x: 100,
    y: 300,
    angle: 0,
    velocity: 0,
    angularVelocity: 0
  });
  
  const [trailerStates, setTrailerStates] = useState<TrailerState[]>([]);
  const [pathPoints, setPathPoints] = useState<Point[]>([]);
  const [trajectory, setTrajectory] = useState<Point[][]>([]);
  
  const [params, setParams] = useState<SimulationParams>({
    dt: 0.05,
    velocity: 10,
    steeringAngle: 0,
    wheelbase: 3.0,
    trackWidth: 1.8,
    trailerCount: 2,
    hitchLength: 1.5,
    trailerWheelbase: 2.5
  });

  const [useStanleyControl, setUseStanleyControl] = useState(false);

  // Initialize trailers
  useEffect(() => {
    const newTrailers: TrailerState[] = [];
    for (let i = 0; i < params.trailerCount; i++) {
      newTrailers.push({
        x: vehicleState.x - (i + 1) * (params.hitchLength + params.trailerWheelbase),
        y: vehicleState.y,
        angle: vehicleState.angle,
        velocity: 0,
        angularVelocity: 0,
        hitchLength: params.hitchLength,
        wheelbase: params.trailerWheelbase
      });
    }
    setTrailerStates(newTrailers);
    
    // Initialize trajectory arrays
    const newTrajectory = Array(params.trailerCount + 1).fill(null).map(() => []);
    setTrajectory(newTrajectory);
  }, [params.trailerCount, params.hitchLength, params.trailerWheelbase]);

  // Simulation step
  const simulationStep = useCallback(() => {
    setVehicleState(prevVehicle => {
      setTrailerStates(prevTrailers => {
        // Ensure prevTrailers is an array
        if (!Array.isArray(prevTrailers)) {
          return prevTrailers;
        }

        let currentSteeringAngle = params.steeringAngle;
        
        // Use Stanley controller if enabled and path exists
        if (useStanleyControl && pathPoints.length > 0) {
          currentSteeringAngle = StanleyController.calculateSteeringAngle(
            prevVehicle,
            pathPoints
          );
        }

        // Update vehicle
        const newVehicle = NumericalIntegrator.updateVehicleState(
          prevVehicle,
          params.velocity,
          currentSteeringAngle,
          params.dt,
          params.wheelbase
        );

        // Update trailers
        const newTrailers = [...prevTrailers];
        let prevState: VehicleState = newVehicle;

        for (let i = 0; i < newTrailers.length; i++) {
          newTrailers[i] = NumericalIntegrator.updateTrailerState(
            newTrailers[i],
            prevState,
            params.dt
          );
          prevState = newTrailers[i];
        }

        // Apply jackknife prevention
        const allAngles = [newVehicle.angle, ...newTrailers.map(t => t.angle)];
        const constrainedAngles = JackknifeController.preventJackknife(allAngles);
        
        newVehicle.angle = constrainedAngles[0];
        for (let i = 0; i < newTrailers.length; i++) {
          newTrailers[i].angle = constrainedAngles[i + 1];
        }

        // Update trajectory
        setTrajectory(prevTrajectory => {
          const newTrajectory = [...prevTrajectory];
          if (newTrajectory[0]) {
            newTrajectory[0] = [...newTrajectory[0], { x: newVehicle.x, y: newVehicle.y }];
          } else {
            newTrajectory[0] = [{ x: newVehicle.x, y: newVehicle.y }];
          }
          
          for (let i = 0; i < newTrailers.length; i++) {
            if (newTrajectory[i + 1]) {
              newTrajectory[i + 1] = [...newTrajectory[i + 1], { x: newTrailers[i].x, y: newTrailers[i].y }];
              // Limit trajectory length
              if (newTrajectory[i + 1].length > 500) {
                newTrajectory[i + 1] = newTrajectory[i + 1].slice(-500);
              }
            } else {
              newTrajectory[i + 1] = [{ x: newTrailers[i].x, y: newTrailers[i].y }];
            }
          }
          return newTrajectory;
        });

        return newTrailers;
      });

      return prevVehicle;
    });
  }, [params, pathPoints, useStanleyControl]);

  // Animation loop
  useEffect(() => {
    if (isRunning) {
      const animate = () => {
        simulationStep();
        animationRef.current = requestAnimationFrame(animate);
      };
      animationRef.current = requestAnimationFrame(animate);
    } else {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    }

    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, [isRunning, simulationStep]);

  // Canvas drawing
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw trajectory
    trajectory.forEach((traj, index) => {
      if (traj.length > 1) {
        ctx.strokeStyle = index === 0 ? '#ff0000' : `hsl(${(index - 1) * 60}, 70%, 50%)`;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(traj[0].x, traj[0].y);
        for (let i = 1; i < traj.length; i++) {
          ctx.lineTo(traj[i].x, traj[i].y);
        }
        ctx.stroke();
      }
    });

    // Draw path points
    if (pathPoints.length > 0) {
      ctx.strokeStyle = '#00ff00';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(pathPoints[0].x, pathPoints[0].y);
      for (let i = 1; i < pathPoints.length; i++) {
        ctx.lineTo(pathPoints[i].x, pathPoints[i].y);
      }
      ctx.stroke();

      // Draw path points
      ctx.fillStyle = '#00ff00';
      pathPoints.forEach(point => {
        ctx.beginPath();
        ctx.arc(point.x, point.y, 3, 0, 2 * Math.PI);
        ctx.fill();
      });
    }

    // Draw vehicle function
    const drawVehicle = (state: VehicleState, color: string, wheelbase: number) => {
      ctx.save();
      ctx.translate(state.x, state.y);
      ctx.rotate(state.angle);
      
      // Vehicle body
      ctx.fillStyle = color;
      ctx.fillRect(-wheelbase/2, -15, wheelbase, 30);
      
      // Direction indicator
      ctx.fillStyle = '#ffffff';
      ctx.fillRect(wheelbase/2 - 10, -5, 15, 10);
      
      ctx.restore();
    };

    // Draw main vehicle
    drawVehicle(vehicleState, '#ff0000', params.wheelbase);

    // Draw trailers
    trailerStates.forEach((trailer, index) => {
      const color = `hsl(${index * 60}, 70%, 50%)`;
      drawVehicle(trailer, color, trailer.wheelbase);
    });

  }, [vehicleState, trailerStates, trajectory, pathPoints, params.wheelbase]);

  // Handle canvas click to set path
  const handleCanvasClick = useCallback((event: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    setPathPoints(prev => [...prev, { x, y }]);
  }, []);

  return (
    <div className="simulator-container" style={{ padding: '20px', fontFamily: 'Arial, sans-serif' }}>
      <h1>多連結トレーラー軌跡シミュレーター</h1>
      
      <div style={{ display: 'flex', gap: '20px' }}>
        <div>
          <h3>制御パネル</h3>
          
          <div style={{ marginBottom: '10px' }}>
            <button 
              onClick={() => setIsRunning(!isRunning)}
              style={{ padding: '10px 20px', fontSize: '16px' }}
            >
              {isRunning ? '停止' : '開始'}
            </button>
            <button 
              onClick={() => {
                setTrajectory(Array(params.trailerCount + 1).fill(null).map(() => []));
                setPathPoints([]);
              }}
              style={{ padding: '10px 20px', fontSize: '16px', marginLeft: '10px' }}
            >
              クリア
            </button>
          </div>

          <div style={{ marginBottom: '10px' }}>
            <label>
              <input
                type="checkbox"
                checked={useStanleyControl}
                onChange={(e) => setUseStanleyControl(e.target.checked)}
              />
              Stanley制御を使用
            </label>
          </div>

          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '10px', marginBottom: '10px' }}>
            <label>
              速度 (m/s):
              <input
                type="range"
                min="0"
                max="20"
                step="0.5"
                value={params.velocity}
                onChange={(e) => setParams(prev => ({ ...prev, velocity: parseFloat(e.target.value) }))}
              />
              <span>{params.velocity}</span>
            </label>

            <label>
              操舵角 (度):
              <input
                type="range"
                min="-30"
                max="30"
                step="1"
                value={params.steeringAngle}
                onChange={(e) => setParams(prev => ({ ...prev, steeringAngle: parseFloat(e.target.value) }))}
                disabled={useStanleyControl}
              />
              <span>{params.steeringAngle}</span>
            </label>

            <label>
              ホイールベース (m):
              <input
                type="range"
                min="1"
                max="5"
                step="0.1"
                value={params.wheelbase}
                onChange={(e) => setParams(prev => ({ ...prev, wheelbase: parseFloat(e.target.value) }))}
              />
              <span>{params.wheelbase}</span>
            </label>

            <label>
              トラック幅 (m):
              <input
                type="range"
                min="1"
                max="3"
                step="0.1"
                value={params.trackWidth}
                onChange={(e) => setParams(prev => ({ ...prev, trackWidth: parseFloat(e.target.value) }))}
              />
              <span>{params.trackWidth}</span>
            </label>

            <label>
              トレーラー数:
              <input
                type="range"
                min="1"
                max="5"
                step="1"
                value={params.trailerCount}
                onChange={(e) => setParams(prev => ({ ...prev, trailerCount: parseInt(e.target.value) }))}
              />
              <span>{params.trailerCount}</span>
            </label>

            <label>
              ヒッチ長 (m):
              <input
                type="range"
                min="0.5"
                max="3"
                step="0.1"
                value={params.hitchLength}
                onChange={(e) => setParams(prev => ({ ...prev, hitchLength: parseFloat(e.target.value) }))}
              />
              <span>{params.hitchLength}</span>
            </label>
          </div>

          <div style={{ marginTop: '20px', fontSize: '12px' }}>
            <h4>技術的改善点:</h4>
            <ul style={{ paddingLeft: '20px' }}>
              <li>✓ 基本運動学モデルの修正（速度表記の明確化）</li>
              <li>✓ トレーラー連結運動学の修正（ベクトル演算）</li>
              <li>✓ アッカーマン幾何学の修正</li>
              <li>✓ トレーラー角速度計算の修正</li>
              <li>✓ Stanley制御器の改善</li>
              <li>✓ ジャックナイフ防止の改善</li>
              <li>✓ 数値安定性の改善（中点法）</li>
            </ul>
            <p style={{ marginTop: '10px' }}>
              <strong>使用方法:</strong> キャンバスをクリックして経路を設定し、Stanley制御を有効にしてください。
            </p>
          </div>
        </div>

        <div>
          <canvas
            ref={canvasRef}
            width={800}
            height={600}
            onClick={handleCanvasClick}
            style={{ border: '1px solid #ccc', cursor: 'crosshair' }}
          />
        </div>
      </div>
    </div>
  );
};

export default MultiTrailerSimulator;