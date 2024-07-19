import { TestBed } from '@angular/core/testing';
import { InfoService } from './info.service';
import { RobotInfo } from '../../interfaces/RobotInfo';
import { take } from 'rxjs/operators';

describe('InfoService', () => {
  let service: InfoService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(InfoService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should update and get robot information correctly', () => {
    const testRobotId = 'robot1';
    const testRobotInfo: RobotInfo = {
      robot_id: testRobotId,
      status: 'active',
      battery_voltage: 12.5,
      start_time: 0,
      position_x: 0,
      position_y: 0,
      init_position: { x: 0, y: 0 },
      init_orientation: { x: 0, y: 0, z: 0, w: 0 }
    };

    service.updateRobotInfo(testRobotId, testRobotInfo);

    expect(service.robotInfos.get(testRobotId)).toEqual(testRobotInfo);
  });

  it('should emit updated robotInfos through observable on update', (done: DoneFn) => {
    const testRobotId = 'robot2';
    const testRobotInfo: RobotInfo = {
      robot_id: testRobotId,
      status: 'inactive',
      battery_voltage: 11.1,
      start_time: 0,
      position_x: 0,
      position_y: 0,
      init_position: { x: 0, y: 0 },
      init_orientation: { x: 0, y: 0, z: 0, w: 0 }
    };
  
    service.updateRobotInfo(testRobotId, testRobotInfo);
  
    service.robotInfosObservable.pipe(take(1)).subscribe((robotInfos) => {
      const updatedRobotInfo = robotInfos.get(testRobotId);
      if (updatedRobotInfo) {
        expect(updatedRobotInfo).toEqual(testRobotInfo);
        done();
      } else {
        fail('Expected robot info to be defined');
      }
    });
  });
  
  
  it('should emit updated map info through observable on update', (done: DoneFn) => {
    const testMapInfo = 'data:image/png;base64,abcd1234';
  
    service.mapInfo = testMapInfo;
  
    service.mapInfoObservable.pipe(take(1)).subscribe((mapInfo) => {
      expect(mapInfo).toEqual(testMapInfo);
      done();
    });
  });
  
});
