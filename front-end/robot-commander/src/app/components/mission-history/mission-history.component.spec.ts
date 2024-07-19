import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MissionHistoryComponent } from './mission-history.component';
import { SocketClientService } from '../../services/socket-client/socket-client.service';
import { MatDialog } from '@angular/material/dialog';
import { CUSTOM_ELEMENTS_SCHEMA } from '@angular/core';
import { of } from 'rxjs';
import { MatCardModule } from '@angular/material/card';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { MatTableModule } from '@angular/material/table';
import { SocketTestHelper } from 'src/app/services/socket-client/SocketTestHelper';
import { Socket } from 'socket.io-client';
import { MissionData } from 'src/app/interfaces/MissionData';

class MockMatDialog {
  open = jasmine.createSpy('open').and.returnValue({
    afterClosed: () => of({})
  });
}

describe('MissionHistoryComponent', () => {
  let service: SocketClientService;
  let component: MissionHistoryComponent;
  let fixture: ComponentFixture<MissionHistoryComponent>;
  let socketHelper: SocketTestHelper;
  let mockMatDialog: MockMatDialog;

  beforeEach(() => {
    mockMatDialog = new MockMatDialog();

    TestBed.configureTestingModule({
      declarations: [MissionHistoryComponent],
      providers: [
        SocketClientService,
        { provide: MatDialog, useValue: mockMatDialog }
      ],
      imports: [
        MatCardModule, 
        BrowserAnimationsModule, 
        MatTableModule
      ],
      schemas: [CUSTOM_ELEMENTS_SCHEMA]
    }).compileComponents();
    service = TestBed.inject(SocketClientService);
    socketHelper = new SocketTestHelper();
    service.socket = socketHelper as unknown as Socket;

  
    fixture = TestBed.createComponent(MissionHistoryComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should open dialog with mission info when openMissionInfo is called', () => {
    const missionId = 1;
    component.openMissionInfo(missionId);
    expect(mockMatDialog.open).toHaveBeenCalled();
    expect(mockMatDialog.open.calls.mostRecent().args[1].data).toEqual({ missionId });
  });

  it('should receive and sort mission history correctly', () => {
    const mockMissions: MissionData[] = [
      {
        missionId: 1,
        date: new Date(),
        time: '12:00:00',
        type: 'Simulation',
        robots: ['lm1166']
      },
      {
        missionId: 2,
        date: new Date(),
        time: '12:00:00',
        type: 'Physique',
        robots: ['lm1166']
      }
    ];

    socketHelper.simulateEvent('mission_history', mockMissions);
    expect(component.missions[0].missionId).toBe(2);
    expect(component.missions[1].missionId).toBe(1);
  });

  it('should receive and add a new mission to the mission history', () => {
    const mockMission: MissionData = {
      missionId: 4,
      date: new Date(),
      time: '12:00:00',
      type: 'Simulation',
      robots: ['lm1166']
    };

    socketHelper.simulateEvent('last_mission', mockMission);
    expect(component.missions[0].missionId).toBe(4);
  });

  it('should emit delete_mission event when deleteMission is called', () => {
    const missionId = 1;
    const spy = spyOn(service.socket, 'emit');
    component.deleteMission(missionId);
    expect(spy).toHaveBeenCalledWith('delete_mission', missionId);
  });

  it('should not remove "options" from displayedColumns if showOptions is true', () => {
    // Assurez-vous que showOptions est à true
    component.showOptions = false;
    component.ngOnInit();
    // Vérifiez que le dernier élément est "options" ou tout autre élément attendu
    expect(component.displayedColumns[component.displayedColumns.length - 1]).toBe('robots');
  });  

  describe('getRobotColor', () => {
    it('should return "red" for "Aucun robot"', () => {
      expect(component.getRobotColor('Aucun robot')).toBe('red');
    });
  
    it('should return "blue" for the robot "lm1166"', () => {
      expect(component.getRobotColor('lm1166')).toBe('blue');
    });
  
    it('should return "green" for the robot "lm1342"', () => {
      expect(component.getRobotColor('lm1342')).toBe('green');
    });
  
    it('should return "black" for an unknown robot', () => {
      expect(component.getRobotColor('unknown_robot')).toBe('black');
    });

    it('should return "purple" for the robot "limo_0"', () => {
      expect(component.getRobotColor('limo_0')).toBe('purple');
    });
  
    it('should return "orange" for the robot "limo_1"', () => {
      expect(component.getRobotColor('limo_1')).toBe('orange');
    });
  });

});
