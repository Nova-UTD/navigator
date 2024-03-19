export type Node = {
	package: string;
	executable: string;
};

export type LaunchListEntry = {
	path: string;
	metadata: {
		name: string;
	};
	nodes: Node[];
};

export type LaunchListResponse = LaunchListEntry[];

export async function launchList(launchDirectory: string): Promise<LaunchListResponse> {
	const response: Promise<LaunchListResponse> = fetch(
		`http://127.0.0.1:8000/launches?dir=${launchDirectory}`
	).then((r) => r.json());

	return response;
}

export type PackageInfo = {
	name: string;
	executables: string[];
};

export type SubsystemInfo = {
	name: string;
	packages: PackageInfo[];
};

export type SubsystemResponse = SubsystemInfo[];

export async function subsystemList(): Promise<SubsystemResponse> {
	const response = fetch('http://127.0.0.1:8000/subsystems').then((r) => r.json());
	return response;
}
