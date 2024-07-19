import { CommandService } from "@src/services/command.service";
import Logger from "@src/utils/Logger";
import { StatusCodes } from "http-status-codes";
import { expect } from "chai";
import { SinonStubbedInstance, createStubInstance } from "sinon";
import * as supertest from "supertest";
import { Container } from "typedi";
import { Application } from '../app';

describe("CommandController", () => {
    let commandServiceMock: SinonStubbedInstance<CommandService>;
    let logger: SinonStubbedInstance<Logger>;
    let expressApp: Express.Application;

    beforeEach(async () => {
        commandServiceMock = createStubInstance(CommandService);
        logger = createStubInstance(Logger);

        const app = Container.get(Application);
        Object.defineProperty(app['commandController'], 'commandService', { value: commandServiceMock });
        Object.defineProperty(app['commandController'], 'LOGGER', { value: logger});
        expressApp = app.app; 
    });

    it("should respond with status 200 when identification is successful", async () => {
        const response = await supertest(expressApp)
            .post('/api/command/identify')
            .send({ robot_id: 'all' });
        expect(response.status).to.equal(StatusCodes.OK);
    });

    it("should respond with status 500 when an error occurs during identification", async () => {
        commandServiceMock.identify.rejects(new Error("Identification failed"));

        const response = await supertest(expressApp)
            .post('/api/command/identify')
            .send({ robot_id: 'all' });
        expect(response.status).to.equal(StatusCodes.INTERNAL_SERVER_ERROR);
    });



    it("should respond with status 200 when starting the mission is successful", async () => {
        const response = await supertest(expressApp)
            .post('/api/command/start_mission');
        expect(response.status).to.equal(StatusCodes.OK);
    });

    it("should respond with status 500 when an error occurs during mission start", async () => {
        commandServiceMock.startMission.rejects(new Error("Mission start failed"));

        const response = await supertest(expressApp)
            .post('/api/command/start_mission');
        expect(response.status).to.equal(StatusCodes.INTERNAL_SERVER_ERROR);
    });



    it("should respond with status 200 when stopping the mission is successful", async () => {
        const response = await supertest(expressApp)
            .post('/api/command/stop_mission');
        expect(response.status).to.equal(StatusCodes.OK);
    });

    it("should respond with status 500 when an error occurs during mission stop", async () => {

        commandServiceMock.stopMission.rejects(new Error("Mission stop failed"));

        const response = await supertest(expressApp)
            .post('/api/command/stop_mission');
        expect(response.status).to.equal(StatusCodes.INTERNAL_SERVER_ERROR);
    });
});
