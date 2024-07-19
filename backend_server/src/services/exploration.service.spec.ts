import { expect } from 'chai';
import { promises as fs } from 'fs';
import { SinonStub, createStubInstance, stub, restore, SinonStubbedInstance } from 'sinon';
import { ExplorationService } from '@src/services/exploration.service';
import { FileService } from '@src/services/file.service';
import Logger from '@src/utils/Logger';
import { log } from 'console';

describe('ExplorationService', () => {
    let explorationService: ExplorationService;
    let fileServiceStub: SinonStubbedInstance<FileService>;
    let fsReadDirStub: SinonStub;
    let fsReadFileStub: SinonStub;
    let loggerErrStub: SinonStub;

    beforeEach(() => {
        fileServiceStub = createStubInstance(FileService);
        explorationService = new ExplorationService(fileServiceStub);
        fsReadDirStub = stub(fs, 'readdir');
        fsReadFileStub = stub(fs, 'readFile');
        loggerErrStub = stub(Logger.prototype, 'err');
    });

    afterEach(() => {
        restore();
    });

    
    it('should return an array of mission data', async () => {
        const directoryEntries = [
            { name: 'exploration-1', isDirectory: () => true },
            { name: 'exploration-2', isDirectory: () => true },
        ];
        fsReadDirStub.resolves(directoryEntries);

        const result = await explorationService.getAllExplorations();

        expect(result.length).to.equal(2);
    });

    it('should return the data of the last mission', async () => {
        const directoryEntries = [
            { name: 'exploration-1', isDirectory: () => true },
            { name: 'exploration-2', isDirectory: () => true },
        ];
        fsReadDirStub.resolves(directoryEntries);

        const infoContent = JSON.stringify({
            date: '2022-03-15',
            time: '12:00',
            type: 'Type',
            robots: [],
        });

        fsReadFileStub.resolves(infoContent);

        const result = await explorationService.getLastMission();

        expect(result).to.deep.equal({
            missionId: 2,
            date: new Date('2022-03-15'),
            time: '12:00',
            type: 'Type',
            robots: [],
        });
    });

    it('should return null if there are no previous explorations', async () => {
        const directoryEntries = [];
        fsReadDirStub.resolves(directoryEntries);

        fsReadFileStub.resolves({});

        const result = await explorationService.getLastMission();

        expect(result).to.be.null;
    });
    
    it('should return details of the specified exploration', async () => {
        const explorationNumber = 1;
        const explorationFolderPath = `./src/assets/explorations/exploration-${explorationNumber}`;

        const logContent = JSON.stringify('Log content');
        const infoContent = JSON.stringify({
            date: '2022-03-15',
            time: '12:00',
            type: 'Type',
            robots: [],
        });

        const expectedDetails = {
            log: logContent,
            info: {
                date: '2022-03-15',
                time: '12:00',
                type: 'Type',
                robots: [],
            },
        };

        fsReadFileStub.withArgs(`${explorationFolderPath}/log.txt`).resolves(logContent);
        fsReadFileStub.withArgs(`${explorationFolderPath}/info.json`).resolves(infoContent);

        const result = await explorationService.getExplorationDetails(explorationNumber);

        expect(result.log).to.deep.equal(expectedDetails.log);
        expect(result.info).to.deep.equal(expectedDetails.info);
    });

});
