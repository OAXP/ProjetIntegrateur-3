import { expect } from 'chai';
import { Dirent, promises as fs } from 'fs';
import { SinonStub, createStubInstance, stub, SinonStubbedInstance, restore } from 'sinon';
import { FileService } from '@src/services/file.service';
import * as rclnodejs from 'rclnodejs';

describe('FileService', () => {
    let fileService: FileService;
    let fsMkdirStub: SinonStub;
    let fsWriteFileStub: SinonStub;
    let fsAppendFileStub: SinonStub;
    let fsReaddirStub: SinonStub;
    let fsRmdirStub: SinonStub;

    beforeEach(() => {
        fsMkdirStub = stub(fs, 'mkdir').resolves();
        fsWriteFileStub = stub(fs, 'writeFile').resolves();
        fsAppendFileStub = stub(fs, 'appendFile').resolves();
        fsReaddirStub = stub(fs, 'readdir').resolves([]);
        fsRmdirStub = stub(fs, 'rmdir').resolves();
        fileService = new FileService();
    });

    afterEach(() => {
        restore();
    });

    it('should start exploration and write info and log files', async () => {
        const robotInfo = new Map<string, rclnodejs.communication_interfaces.msg.RobotInfo>();
        robotInfo.set('limo_1',{
            robot_id: 'limo_test',
            status: 'En attente de mission',
            battery_voltage: 12.5,
            start_time: 0,
            position_x: 0,
            position_y: 0,
            distance: 0,
        });
        
        await fileService.startExploration(robotInfo);

        expect(fsMkdirStub.calledOnce).to.be.true;
        expect(fsWriteFileStub.calledTwice).to.be.true;
    });

    it('should set type to "Physique" if robot names do not start with "limo"', async () => {
        const robotInfo = new Map<string, rclnodejs.communication_interfaces.msg.RobotInfo>();
        robotInfo.set('lm_test',{
            robot_id: 'lm_test',
            status: 'En attente de mission',
            battery_voltage: 12.5,
            start_time: 0,
            position_x: 0,
            position_y: 0,
            distance: 0,
        });
        
        await fileService.startExploration(robotInfo);

        const expectedInfo = {
            missionId: 'exploration-1',
            date: new Date().toISOString().split('T')[0],
            time: new Date().toLocaleTimeString('fr-CA', { hour12: false }),
            robots: ['lm_test'],
            type: 'Physique',
        };

        expect(fsWriteFileStub.calledWith('./src/assets/explorations/exploration-1/info.json', JSON.stringify(expectedInfo))).to.be.true;
    });

    it('should end exploration and clear log folder path', () => {
        fileService.endExploration();

        expect(fileService['logFolderPath']).to.equal('');
    });

    it('should write log message to log file if exploration is ongoing', () => {
        fileService['logFolderPath'] = 'test/log.txt';

        fileService.writeLog('Test log message');

        expect(fsAppendFileStub.calledOnceWith('test/log.txt', 'Test log message\n')).to.be.true;
    });

    it('should not write log message if exploration is not ongoing', () => {
        fileService.writeLog('Test log message');

        expect(fsAppendFileStub.notCalled).to.be.true;
    });

    it('should delete exploration directory', async () => {
        await fileService.deleteExploration(1);

        expect(fsRmdirStub.calledOnceWith('./src/assets/explorations/exploration-1', { recursive: true })).to.be.true;
    });

    it('should handle case where no exploration directories exist', async () => {
        const nextNumber = await fileService['getExplorationNumber']();

        expect(nextNumber).to.equal(1);
    });

    it('should return false', () => {
        fileService['logFolderPath'] = '';
        
        expect(fileService.isExploring).to.be.false;
    });

    it('should get the next available exploration number', async () => {
        const dirent1 = new Dirent();
        dirent1.name = 'exploration-0';
        dirent1.isDirectory = () => true;
    
        const dirent2 = new Dirent();
        dirent2.name = 'exploration-1';
        dirent2.isDirectory = () => true;
    
        fsReaddirStub.resolves([dirent1, dirent2]);
    
        const nextNumber = await fileService['getExplorationNumber']();
    
        expect(nextNumber).to.equal(2);
    });


    it('should return 1 if no valid directories', async () => {
        const dirent1 = new Dirent();
        dirent1.name = 'invalid_folder';
        dirent1.isDirectory = () => true;
        
        fsReaddirStub.resolves([dirent1]);
    
        const nextNumber = await fileService['getExplorationNumber']();
    
        expect(nextNumber).to.equal(1);
    });
});
