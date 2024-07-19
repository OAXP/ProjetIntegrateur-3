import { expect } from 'chai';
import { createStubInstance, stub, restore } from 'sinon';
import { InfoService } from '@src/services/info.service';
import { RosService } from '@src/services/ros.service';
import { FileService } from '@src/services/file.service';
import { SocketManagerService } from '@src/services/socket-manager.service';
import * as rclnodejs from 'rclnodejs';

describe('InfoService', () => {
    let infoService: InfoService;
    let rosServiceStub: sinon.SinonStubbedInstance<RosService>;
    let fileServiceMock: sinon.SinonStubbedInstance<FileService>;
    let socketManagerServiceMock: sinon.SinonStubbedInstance<SocketManagerService>;

    beforeEach(() => {
        rosServiceStub = createStubInstance(RosService);
        rosServiceStub.getNode.returns({
            createSubscription: stub().returnsThis(),
        } as unknown as rclnodejs.Node);
        fileServiceMock = createStubInstance(FileService);
        socketManagerServiceMock = createStubInstance(SocketManagerService);

        infoService = new InfoService(rosServiceStub, socketManagerServiceMock, fileServiceMock);
    });

    afterEach(() => {
        restore();
    });
    
    it('should create a subscription to /robot_info topic', () => {
        infoService.init();
    
        expect(rosServiceStub.getNode.calledOnce).to.be.true;
    });
    
    it('should add robot information to the robots map', () => {
        const msg = {
            robot_id: 'lm_test',
            status: 'En attente de mission',
            battery_voltage: 12.5,
            start_time: 0,
            position_x: 0,
            position_y: 0,
            distance: 0,
        };

        infoService.onRobotInfo(msg);

        expect(infoService.robotInfo.size).to.equal(1);
    });
    
});
