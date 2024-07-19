import { ComponentFixture, TestBed } from '@angular/core/testing';

import { InitPositionComponent } from './init-position.component';
import { CommunicationService } from 'src/app/services/communication/communication.service';
import { of } from 'rxjs';
import { MatCardModule } from '@angular/material/card';
import { MatRadioModule } from '@angular/material/radio';
import { MatFormFieldModule } from '@angular/material/form-field';
import { FormsModule } from '@angular/forms';
import { Quaternion } from 'src/app/interfaces/Quaternion';

class MockCommunicationService {
  startMission = jasmine.createSpy('startMission').and.returnValue(of({}));
  stopMission = jasmine.createSpy('stopMission').and.returnValue(of({}));
  setInitPositions = jasmine.createSpy('setInitPositions').and.returnValue(of({}));
}

describe('InitPositionComponent', () => {
  let component: InitPositionComponent;
  let fixture: ComponentFixture<InitPositionComponent>;
  let mockCommunicationService: MockCommunicationService;

  beforeEach(async () => {
    mockCommunicationService = new MockCommunicationService();

    await TestBed.configureTestingModule({
      declarations: [InitPositionComponent],
      providers: [
        { provide: CommunicationService, useValue: mockCommunicationService }
      ],
      imports: [MatCardModule, MatRadioModule, MatFormFieldModule, FormsModule]
    })
    .compileComponents();
    
    fixture = TestBed.createComponent(InitPositionComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should update selected robot initial position and orientation on origin robot change', () => {
    component.originRobotId = 'limo_1';
    component.onOriginRobotChange();
    const selectedRobot = component['_robots'].find(robot => robot.robot_id === component.originRobotId);
    expect(selectedRobot?.init_position).toEqual({ x: 0, y: 0 });
    expect(selectedRobot?.init_orientation).toEqual({ x: 0, y: 0, z: 0, w: 1 });
  });

  it('should update non-selected robot position and orientation', () => {
    component.originRobotId = 'limo_0';
    component.updateNonSelectedRobot();
    expect(component.nonSelectedRobotPosition).not.toEqual({ x: 0, y: 1 });
    expect(component.nonSelectedRobotOrientation).not.toEqual({ x: 0, y: 0, z: 0, w: 0 });
  });

  it('should confirm initial positions and set isConfirmed to true', () => {
    component['_robots'] = [{
      robot_id: 'limo_0',
      status: '',
      battery_voltage: 0,
      start_time: 0,
      init_position: { x: 1, y: 2 },
      init_orientation: { x: 0.1, y: 0.2, z: 0.3, w: 0.4 },
      position_x: 1,
      position_y: 2
    }];
    component.confirmInitPositions();
    expect(mockCommunicationService.setInitPositions).toHaveBeenCalled();
    expect(component['isConfirmed']).toBeTrue();
  });

  it('should update the non-selected robot\'s initial position correctly', () => {
    // Configuration initiale avec deux robots pour garantir qu'il y a un robot non sélectionné
    component['_robots'] = [
      {
        robot_id: 'limo_0', 
        status: 'En attente de mission', 
        battery_voltage: 10, 
        start_time: 0.0, 
        init_position: { x: 0, y: 0 }, 
        init_orientation: { x: 0, y: 0, z: 0, w: 1 },
        position_x: 0,
        position_y: 0
      },
      {
        robot_id: 'limo_1', 
        status: 'En attente de mission', 
        battery_voltage: 10, 
        start_time: 0.0, 
        init_position: { x: 0, y: 0 }, 
        init_orientation: { x: 0, y: 0, z: 0, w: 1 },
        position_x: 0,
        position_y: 0
      }
    ];
    // Sélectionnez 'limo_0' comme robot d'origine pour que 'limo_1' soit le robot non sélectionné
    component.originRobotId = 'limo_0';

    // Testez la mise à jour de la position x
    const newXValue = 5;
    component.updateNonSelectedRobotPosition('x', newXValue);
    expect(component.nonSelectedRobot?.init_position.x).toBe(newXValue);
    expect(component['isConfirmed']).toBeFalse(); // Vérifiez également que isConfirmed est réinitialisé

    // Testez la mise à jour de la position y
    const newYValue = 10;
    component.updateNonSelectedRobotPosition('y', newYValue);
    expect(component.nonSelectedRobot?.init_position.y).toBe(newYValue);
    expect(component['isConfirmed']).toBeFalse();
  });
  
  it('should update the non-selected robot\'s initial orientation correctly', () => {
    // Préparez les données initiales avec deux robots
    component['_robots'] = [
      {
        robot_id: 'limo_0', 
        status: 'En attente de mission', 
        battery_voltage: 10, 
        start_time: 0.0, 
        init_position: { x: 0, y: 0 }, 
        init_orientation: { x: 0, y: 0, z: 0, w: 1 },
        position_x: 0,
        position_y: 0
      },
      {
        robot_id: 'limo_1', 
        status: 'En attente de mission', 
        battery_voltage: 10, 
        start_time: 0.0, 
        init_position: { x: 0, y: 0 }, 
        init_orientation: { x: 0, y: 0, z: 0, w: 1 },
        position_x: 0,
        position_y: 0
      }
    ];
    // Sélectionnez 'limo_0' comme robot d'origine pour que 'limo_1' soit le robot non sélectionné
    component.originRobotId = 'limo_0';

    // Testez la mise à jour de chaque axe de l'orientation
    const testValues = { x: 0.1, y: 0.2, z: 0.3, w: 0.4 };
    Object.entries(testValues).forEach(([axis, value]) => {
      component.updateNonSelectedRobotOrientation(axis as 'x' | 'y' | 'z' | 'w', value);
      expect((component.nonSelectedRobot?.init_orientation as Quaternion)[axis as 'x' | 'y' | 'z' | 'w']).toBe(value);
      expect(component['isConfirmed']).toBeFalse(); // Vérifie que isConfirmed est réinitialisé
    });
  });
  
});
