import { expect } from 'chai';
import { SinonStubbedInstance, createStubInstance, stub, restore } from 'sinon';
import { CommandService } from '@src/services/command.service';
import { RosService } from '@src/services/ros.service';
import { MapService } from '@src/services/map.service';
import { FileService } from '@src/services/file.service';
import { InfoService } from '@src/services/info.service';
import { SocketManagerService } from '@src/services/socket-manager.service';
import * as rclnodejs from 'rclnodejs';
import Logger from "@src/utils/Logger";

describe('CommandService', () => {
    let commandService: CommandService;
    let rosServiceStub: SinonStubbedInstance<RosService>;
    let mapServiceStub: SinonStubbedInstance<MapService>;
    let fileServiceStub: SinonStubbedInstance<FileService>;
    let infoServiceStub: SinonStubbedInstance<InfoService>;
    let socketManagerServiceStub: SinonStubbedInstance<SocketManagerService>;
    let logger: SinonStubbedInstance<Logger>;

    beforeEach(() => {
        rosServiceStub = createStubInstance(RosService);
        rosServiceStub.getNode.resolves({
            createPublisher: stub().returnsThis(),
            destroyPublisher: stub().returnsThis(),
        } as unknown as rclnodejs.Node);
        mapServiceStub = createStubInstance(MapService);
        fileServiceStub = createStubInstance(FileService);
        infoServiceStub = createStubInstance(InfoService);
        logger = createStubInstance(Logger);
        socketManagerServiceStub = createStubInstance(SocketManagerService);
        commandService = new CommandService(
            rosServiceStub,
            mapServiceStub,
            fileServiceStub,
            infoServiceStub,
            socketManagerServiceStub
        );
        Object.defineProperty(commandService, 'LOGGER', {
            value: logger,
            writable: true
        });

    });

    afterEach(() => {
        restore();
    });

    it('should publish identification message for all robots', async () => {
        rosServiceStub.initialized = true;

        await commandService.identify();

        expect(rosServiceStub.getNode.calledOnce).to.be.true;
    });

    it('identify should stop if rosService is not initialized', async () => {
        rosServiceStub.initialized = false;

        await commandService.identify();

        expect(rosServiceStub.getNode.calledOnce).to.be.false;
    });

    it('should start mission', async () => {
        commandService['missionStarted'] = false;
        rosServiceStub.initialized = true;

        await commandService.startMission();

        expect(fileServiceStub.startExploration.calledOnce).to.be.true;
        expect(socketManagerServiceStub.sendLastMission.calledOnce).to.be.true;
        expect(rosServiceStub.getNode.calledOnce).to.be.true;
    });

    it('startMission should stop if rosService is not initialized', async () => {
        rosServiceStub.initialized = false;

        await commandService.startMission();

        expect(rosServiceStub.getNode.calledOnce).to.be.false;
    });

    it('startMission should skip if already exploring', async () => {
        commandService['missionStarted'] = true;
        rosServiceStub.initialized = true;

        await commandService.startMission();

        expect(fileServiceStub.startExploration.calledOnce).to.be.false;
        expect(socketManagerServiceStub.sendLastMission.calledOnce).to.be.false;
        expect(rosServiceStub.getNode.calledOnce).to.be.true;
    });

    it('should stop mission', async () => {
        commandService['missionStarted'] = true;
        rosServiceStub.initialized = true;

        await commandService.stopMission();

        expect(fileServiceStub.endExploration.calledOnce).to.be.true;
    });

    it('stopMission should stop if rosService is not initialized', async () => {
        rosServiceStub.initialized = false;

        await commandService.stopMission();

        expect(rosServiceStub.getNode.calledOnce).to.be.false;
    });
});
